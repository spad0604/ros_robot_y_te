#!/usr/bin/env python3
"""Upload changed ROS map YAML/image pairs to the central API without auth."""

import json
import os
import re
import uuid
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import rospy

DEFAULT_API_BASE_URL = "https://34.172.236.168:8081/api/v1"
MAX_YAML_BYTES = 1024 * 1024
MAX_IMAGE_BYTES = 50 * 1024 * 1024
SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
IMAGE_FIELD = re.compile(r"(?m)^[ \t]*image[ \t]*:[ \t]*([^\r\n#]+)")


def file_signature(path):
    value = os.stat(path)
    return "%d:%d" % (value.st_mtime_ns, value.st_size)


def parse_image_reference(yaml_bytes):
    text = yaml_bytes.decode("utf-8")
    match = IMAGE_FIELD.search(text)
    if not match:
        raise ValueError("map YAML does not have an image: field")
    value = match.group(1).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1]
    return value


def make_multipart(map_name, yaml_name, yaml_bytes, image_name, image_bytes):
    boundary = "----medibot-map-" + uuid.uuid4().hex
    body = bytearray()

    def add_field(name, value):
        body.extend(("--%s\r\n" % boundary).encode("ascii"))
        body.extend((
            'Content-Disposition: form-data; name="%s"\r\n\r\n' % name
        ).encode("ascii"))
        body.extend(value.encode("utf-8"))
        body.extend(b"\r\n")

    def add_file(name, filename, content_type, content):
        body.extend(("--%s\r\n" % boundary).encode("ascii"))
        body.extend((
            'Content-Disposition: form-data; name="%s"; filename="%s"\r\n'
            % (name, filename)
        ).encode("ascii"))
        body.extend(("Content-Type: %s\r\n\r\n" % content_type).encode("ascii"))
        body.extend(content)
        body.extend(b"\r\n")

    add_field("mapName", map_name)
    add_file("mapYaml", yaml_name, "application/x-yaml", yaml_bytes)
    add_file("image", image_name, "application/octet-stream", image_bytes)
    body.extend(("--%s--\r\n" % boundary).encode("ascii"))
    return boundary, bytes(body)


class MapUploader:
    def __init__(self):
        package_maps = "$(find localize_manager)/maps"
        self.maps_dir = os.path.realpath(rospy.get_param("~maps_dir", package_maps))
        self.api_base = rospy.get_param("~api_base_url", DEFAULT_API_BASE_URL).rstrip("/")
        self.robot_code = rospy.get_param("~robot_code", "MEDIBOT-01")
        self.interval = max(1, int(rospy.get_param("~scan_interval_seconds", 3)))
        self.timeout = max(10, int(rospy.get_param("~request_timeout_seconds", 180)))
        self.state_path = os.path.expanduser(
            rospy.get_param("~state_file", "~/.ros/map_uploader_state.json")
        )
        self.state = self._load_state()
        if not self.api_base.startswith(("https://", "http://")):
            raise ValueError("~api_base_url must be an HTTP(S) URL")
        if not re.match(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$", self.robot_code):
            raise ValueError("~robot_code contains unsupported characters")

    def _load_state(self):
        try:
            with open(self.state_path, "r", encoding="utf-8") as stream:
                value = json.load(stream)
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save_state(self):
        directory = os.path.dirname(self.state_path)
        os.makedirs(directory, exist_ok=True)
        temporary = self.state_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(self.state, stream, sort_keys=True)
        os.replace(temporary, self.state_path)

    def _map_pair(self, yaml_path):
        yaml_name = os.path.basename(yaml_path)
        if not SAFE_NAME.match(yaml_name):
            raise ValueError("unsafe YAML filename")
        yaml_before = file_signature(yaml_path)
        with open(yaml_path, "rb") as stream:
            yaml_bytes = stream.read(MAX_YAML_BYTES + 1)
        if not yaml_bytes or len(yaml_bytes) > MAX_YAML_BYTES:
            raise ValueError("YAML is empty or larger than 1 MB")
        if file_signature(yaml_path) != yaml_before:
            raise ValueError("YAML changed while being read; will retry")

        image_reference = parse_image_reference(yaml_bytes)
        image_path = image_reference if os.path.isabs(image_reference) else os.path.join(
            os.path.dirname(yaml_path), image_reference
        )
        image_path = os.path.realpath(image_path)
        if os.path.commonpath((self.maps_dir, image_path)) != self.maps_dir:
            raise ValueError("image path must stay inside the maps directory")
        image_name = os.path.basename(image_path)
        if not SAFE_NAME.match(image_name) or os.path.splitext(image_name)[1].lower() not in (
            ".pgm", ".png", ".bmp"
        ):
            raise ValueError("image must be a safe .pgm, .png, or .bmp file")

        image_before = file_signature(image_path)
        with open(image_path, "rb") as stream:
            image_bytes = stream.read(MAX_IMAGE_BYTES + 1)
        if not image_bytes or len(image_bytes) > MAX_IMAGE_BYTES:
            raise ValueError("image is empty or larger than 50 MB")
        if file_signature(image_path) != image_before:
            raise ValueError("image changed while being read; will retry")

        relative_dir = os.path.relpath(os.path.dirname(yaml_path), self.maps_dir)
        raw_name = os.path.basename(relative_dir) if relative_dir != "." else ""
        if not raw_name:
            raw_name = os.path.splitext(yaml_name)[0]
        map_name = re.sub(r"[^A-Za-z0-9._ -]", "_", raw_name).strip(" ._-")[:128]
        if not map_name:
            map_name = "map"
        signature = "%s|%s" % (yaml_before, image_before)
        return map_name, yaml_name, yaml_bytes, image_name, image_bytes, signature

    def _upload(self, pair):
        map_name, yaml_name, yaml_bytes, image_name, image_bytes, signature = pair
        boundary, body = make_multipart(
            map_name, yaml_name, yaml_bytes, image_name, image_bytes
        )
        url = "%s/maps/%s/upload" % (self.api_base, quote(self.robot_code, safe=""))
        request = Request(
            url,
            data=body,
            headers={
                "Content-Type": "multipart/form-data; boundary=%s" % boundary,
                "Content-Length": str(len(body)),
            },
            method="POST",
        )
        with urlopen(request, timeout=self.timeout) as response:
            response.read(4096)
            if response.status < 200 or response.status >= 300:
                raise RuntimeError("server returned HTTP %s" % response.status)
        return signature

    def scan_once(self):
        if not os.path.isdir(self.maps_dir):
            rospy.logwarn_throttle(60, "Map uploader: maps directory missing: %s", self.maps_dir)
            return
        for root, dirs, files in os.walk(self.maps_dir, followlinks=False):
            dirs.sort()
            for filename in sorted(files):
                if rospy.is_shutdown():
                    return
                if not filename.lower().endswith((".yaml", ".yml")):
                    continue
                yaml_path = os.path.join(root, filename)
                state_key = "%s|%s|%s" % (
                    self.api_base, self.robot_code, os.path.realpath(yaml_path)
                )
                try:
                    pair = self._map_pair(yaml_path)
                    if self.state.get(state_key) == pair[-1]:
                        continue
                    self.state[state_key] = self._upload(pair)
                    self._save_state()
                    rospy.loginfo("Map uploader: uploaded '%s'", pair[0])
                except (OSError, ValueError, HTTPError, URLError, RuntimeError) as error:
                    rospy.logerr("Map uploader: failed for %s: %s", yaml_path, error)

    def run(self):
        rospy.loginfo(
            "Map uploader ready: %s -> %s (robot %s)",
            self.maps_dir, self.api_base, self.robot_code
        )
        while not rospy.is_shutdown():
            self.scan_once()
            try:
                rospy.sleep(self.interval)
            except rospy.ROSInterruptException:
                return


def main():
    rospy.init_node("map_uploader", anonymous=False)
    try:
        MapUploader().run()
    except (OSError, ValueError) as error:
        rospy.logfatal("Map uploader configuration error: %s", error)
        raise


if __name__ == "__main__":
    main()
