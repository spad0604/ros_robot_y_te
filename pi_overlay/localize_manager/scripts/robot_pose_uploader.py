#!/usr/bin/env python3
"""Publish the localized robot pose to the API without touching motion topics."""

import json
import math
import os
import queue
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import rospy
import tf2_ros
from std_msgs.msg import String


class RobotApi:
    def __init__(self, code, base_url, credentials_file, timeout):
        self.code = code
        self.base_url = base_url.rstrip("/")
        self.credentials_file = os.path.expanduser(credentials_file)
        self.timeout = timeout
        self.token = None
        self.robot_path = "/robots/" + quote(code, safe="")

    def _login(self):
        with open(self.credentials_file, encoding="utf-8") as source:
            credentials = json.load(source)
        body = json.dumps({key: credentials[key] for key in ("username", "password")}).encode("utf-8")
        request = Request(
            self.base_url + "/auth/login",
            data=body,
            headers={"Accept": "application/json", "Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=self.timeout) as response:
            data = json.load(response)["data"]
        user = data.get("user", {})
        if user.get("role") != "ROBOT" or user.get("username") != self.code:
            raise ValueError("Pose uploader requires matching robot credentials")
        self.token = data["accessToken"]

    def call(self, path, body=None, retry_auth=True):
        if not self.token:
            self._login()
        headers = {
            "Accept": "application/json",
            "Content-Type": "application/json",
            "Authorization": "Bearer " + self.token,
        }
        request = Request(
            self.base_url + path,
            data=None if body is None else json.dumps(body, allow_nan=False, separators=(",", ":")).encode("utf-8"),
            headers=headers,
            method="POST" if body is not None else "GET",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                return json.load(response).get("data")
        except HTTPError as error:
            if error.code == 401 and retry_auth:
                self.token = None
                return self.call(path, body, retry_auth=False)
            raise

    def current_map(self):
        selection = self.call(self.robot_path + "/map-selection")
        if not isinstance(selection, dict) or str(selection.get("status", "")).upper() != "APPLIED":
            return None, None
        return selection.get("mapId"), selection.get("mapName")

    def publish_pose(self, pose):
        return self.call(self.robot_path + "/pose", pose)


class RobotPoseUploader:
    def __init__(self):
        self.robot_code = str(rospy.get_param("~robot_code", "MEDIBOT-01")).strip()
        self.api_base_url = str(rospy.get_param(
            "~api_base_url", "https://34.172.236.168:8081/api/v1"
        )).rstrip("/")
        self.credentials_file = rospy.get_param(
            "~credentials_file", "~/.config/medibot/order_listener.json"
        )
        self.global_frame = str(rospy.get_param("~global_frame", "map")).lstrip("/")
        self.base_frame = str(rospy.get_param("~base_frame", "base_link")).lstrip("/")
        self.state_topic = str(rospy.get_param("~state_topic", "/localize_manager/state"))
        self.publish_rate_hz = min(5.0, max(0.5, float(rospy.get_param("~publish_rate_hz", 2.0))))
        self.map_refresh_seconds = max(2.0, float(rospy.get_param("~map_refresh_seconds", 5.0)))
        self.request_timeout_seconds = max(1.0, float(rospy.get_param("~request_timeout_seconds", 3.0)))
        if not self.robot_code or not self.api_base_url.startswith(("https://", "http://")):
            raise ValueError("robot_code and HTTP(S) api_base_url are required")

        self.api = RobotApi(
            self.robot_code,
            self.api_base_url,
            self.credentials_file,
            self.request_timeout_seconds,
        )
        self.state = ""
        self.map_id = None
        self.map_name = None
        self.last_map_refresh = 0.0
        self.pending = queue.Queue(maxsize=1)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.state_subscriber = rospy.Subscriber(self.state_topic, String, self._state_changed, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self._capture)
        self.worker = threading.Thread(target=self._upload_loop, name="robot-pose-uploader", daemon=True)
        self.worker.start()
        rospy.loginfo(
            "Robot pose uploader ready: TF %s -> %s at %.1f Hz while LOCALIZING",
            self.global_frame,
            self.base_frame,
            self.publish_rate_hz,
        )

    def _state_changed(self, message):
        self.state = message.data.strip().upper()

    def _capture(self, _event):
        if self.state != "LOCALIZING":
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.15),
            )
            stamped_at = transform.header.stamp
            if stamped_at == rospy.Time() or (rospy.Time.now() - stamped_at).to_sec() > 2.0:
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            x, y = float(translation.x), float(translation.y)
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                return
            sample = {
                "frameId": self.global_frame,
                "x": x,
                "y": y,
                "yaw": yaw,
            }
            try:
                self.pending.put_nowait(sample)
            except queue.Full:
                try:
                    self.pending.get_nowait()
                except queue.Empty:
                    pass
                self.pending.put_nowait(sample)
        except Exception as error:
            rospy.logwarn_throttle(10.0, "Robot map pose unavailable: %s", type(error).__name__)

    def _upload_loop(self):
        while not rospy.is_shutdown():
            try:
                sample = self.pending.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                now = time.monotonic()
                if now - self.last_map_refresh >= self.map_refresh_seconds:
                    self.last_map_refresh = now
                    try:
                        self.map_id, self.map_name = self.api.current_map()
                    except (HTTPError, URLError, OSError, ValueError, KeyError) as error:
                        self.map_id, self.map_name = None, None
                        rospy.logwarn_throttle(10.0, "Could not read applied map metadata: %s", type(error).__name__)
                sample["mapId"] = self.map_id
                sample["mapName"] = self.map_name
                self.api.publish_pose(sample)
            except HTTPError as error:
                rospy.logwarn_throttle(10.0, "Robot pose API rejected update with HTTP %s", error.code)
            except (URLError, OSError, ValueError, KeyError) as error:
                rospy.logwarn_throttle(10.0, "Robot pose upload failed: %s", type(error).__name__)


def main():
    rospy.init_node("robot_pose_uploader", anonymous=False)
    RobotPoseUploader()
    rospy.spin()


if __name__ == "__main__":
    main()
