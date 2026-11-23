#!/usr/bin/env python3
"""Send current, real ROS sensor and motion-state samples to the robot API."""

import datetime
import json
import math
import re
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import rospy
from moving_server_msgs.msg import MovingServerState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, UInt16


DEFAULT_API_BASE_URL = "https://34.172.236.168:8081/api/v1"
SAFE_ROBOT_CODE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def finite_round(value, places=4):
    number = float(value)
    return round(number, places) if math.isfinite(number) else None


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="seconds"
    ).replace("+00:00", "Z")


class SensorTelemetryUploader:
    def __init__(self):
        self.api_base = rospy.get_param(
            "~api_base_url", DEFAULT_API_BASE_URL
        ).rstrip("/")
        self.robot_code = str(rospy.get_param("~robot_code", "MEDIBOT-01"))
        self.interval = max(
            1.0, float(rospy.get_param("~publish_interval_seconds", 2.0))
        )
        self.timeout = max(
            2.0, float(rospy.get_param("~request_timeout_seconds", 8.0))
        )
        self.max_age = max(
            self.interval, float(rospy.get_param("~max_sensor_age_seconds", 15.0))
        )
        self.front_angle = float(
            rospy.get_param("~front_angle_rad", math.pi)
        )
        self.front_half_angle = math.radians(
            max(1.0, min(90.0, float(rospy.get_param("~front_half_angle_degrees", 30.0))))
        )
        if not self.api_base.startswith(("https://", "http://")):
            raise ValueError("~api_base_url must be an HTTP(S) URL")
        if not SAFE_ROBOT_CODE.match(self.robot_code):
            raise ValueError("~robot_code contains unsupported characters")

        self.endpoint = "%s/robots/%s/telemetry" % (
            self.api_base, quote(self.robot_code, safe="")
        )
        self._lock = threading.Lock()
        self._latest = {}
        self._has_uploaded = False

        odom_topic = rospy.get_param("~odom_topic", "/odom")
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        rpm_topic = rospy.get_param("~lidar_rpm_topic", "/rpms")
        state_topic = rospy.get_param("~motion_state_topic", "/moving_server/state")
        esp32_sensor_topic = rospy.get_param(
            "~esp32_sensor_topic", "/esp32/sensor_data"
        )
        self._subscribers = [
            rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=1),
            rospy.Subscriber(scan_topic, LaserScan, self._on_scan, queue_size=1),
            rospy.Subscriber(rpm_topic, UInt16, self._on_rpm, queue_size=1),
            rospy.Subscriber(
                state_topic, MovingServerState, self._on_motion_state, queue_size=1
            ),
            rospy.Subscriber(
                esp32_sensor_topic,
                Float32MultiArray,
                self._on_esp32_sensor,
                queue_size=1,
            ),
        ]
        rospy.loginfo(
            "Telemetry uploader ready for %s (%s); ROS inputs: %s, %s, %s, %s, %s",
            self.robot_code, self.api_base, odom_topic, scan_topic, rpm_topic,
            state_topic, esp32_sensor_topic
        )

    def _store(self, key, value):
        with self._lock:
            self._latest[key] = (time.monotonic(), value)

    def _on_odom(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        values = {
            "xM": finite_round(pose.position.x),
            "yM": finite_round(pose.position.y),
            "yawRad": finite_round(yaw),
            "linearXMS": finite_round(twist.linear.x),
            "linearYMS": finite_round(twist.linear.y),
            "angularZRadS": finite_round(twist.angular.z),
        }
        self._store("odom", {key: value for key, value in values.items() if value is not None})

    def _on_scan(self, message):
        front_ranges = []
        all_ranges = []
        angle = message.angle_min
        for distance in message.ranges:
            distance = float(distance)
            if (
                math.isfinite(distance)
                and distance >= message.range_min
                and distance <= message.range_max
            ):
                all_ranges.append(distance)
                relative_angle = math.atan2(
                    math.sin(angle - self.front_angle),
                    math.cos(angle - self.front_angle),
                )
                if abs(relative_angle) <= self.front_half_angle:
                    front_ranges.append(distance)
            angle += message.angle_increment

        lidar = {"validSamples": len(all_ranges)}
        if front_ranges:
            lidar["frontNearestM"] = finite_round(min(front_ranges), 3)
            lidar["frontValidSamples"] = len(front_ranges)
        if all_ranges:
            lidar["nearestM"] = finite_round(min(all_ranges), 3)
        range_min = finite_round(message.range_min, 3)
        range_max = finite_round(message.range_max, 3)
        if range_min is not None:
            lidar["rangeMinM"] = range_min
        if range_max is not None:
            lidar["rangeMaxM"] = range_max
        self._store("lidar", lidar)

    def _on_rpm(self, message):
        self._store("lidarRpm", int(message.data))

    def _on_motion_state(self, message):
        self._store(
            "motion",
            {
                "active": bool(message.active),
                "mode": int(message.mode),
                "modeName": str(message.mode_name),
                "state": int(message.state),
                "stateName": str(message.state_name),
                "message": str(message.message),
            },
        )

    def _on_esp32_sensor(self, message):
        # Keep this order in sync with nox/scripts/sensor.py.
        field_names = (
            "temperatureC",
            "humidityPercent",
            "mqAdc",
            "batteryAdc",
            "batteryVoltageV",
            "dustAdc",
            "dustVoltageV",
            "dustDensity",
        )
        if len(message.data) < len(field_names):
            rospy.logwarn_throttle(
                30,
                "Telemetry uploader: %s needs 8 values, got %s",
                "/esp32/sensor_data",
                len(message.data),
            )
            return

        sensors = {
            name: finite_round(message.data[index], 3)
            for index, name in enumerate(field_names)
        }
        self._store(
            "esp32Sensors",
            {name: value for name, value in sensors.items() if value is not None},
        )

    def _snapshot(self):
        now = time.monotonic()
        with self._lock:
            latest = dict(self._latest)
        values = {}
        for key in ("odom", "lidar", "motion", "esp32Sensors"):
            entry = latest.get(key)
            if entry is not None and now - entry[0] <= self.max_age:
                values[key] = dict(entry[1])
        rpm = latest.get("lidarRpm")
        if "lidar" in values and rpm is not None and now - rpm[0] <= self.max_age:
            values["lidar"]["motorRpm"] = rpm[1]
        if not values:
            return None
        values["sampledAtUtc"] = utc_now()
        values["source"] = "ros"
        return values

    def upload_once(self):
        values = self._snapshot()
        if values is None:
            rospy.logwarn_throttle(
                30, "Telemetry uploader: waiting for live ROS sensor/state messages"
            )
            return None

        body = json.dumps(
            values, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        ).encode("utf-8")
        request = Request(
            self.endpoint,
            data=body,
            headers={
                "Content-Type": "application/json; charset=utf-8",
                "Accept": "application/json",
            },
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                response.read(2048)
                if response.status < 200 or response.status >= 300:
                    raise RuntimeError("server returned HTTP %s" % response.status)
            if not self._has_uploaded:
                rospy.loginfo("Telemetry uploader: first live sample accepted by server")
                self._has_uploaded = True
            return True
        except (HTTPError, URLError, OSError, RuntimeError, ValueError) as error:
            rospy.logwarn_throttle(
                30, "Telemetry uploader: upload failed; will retry: %s", error
            )
            return False

    def run(self):
        retry_delay = self.interval
        while not rospy.is_shutdown():
            result = self.upload_once()
            if result is True:
                retry_delay = self.interval
            elif result is False:
                retry_delay = min(max(self.interval, retry_delay * 2.0), 30.0)
            else:
                retry_delay = self.interval
            try:
                rospy.sleep(retry_delay)
            except rospy.ROSInterruptException:
                return


def main():
    rospy.init_node("sensor_telemetry_uploader", anonymous=False)
    try:
        SensorTelemetryUploader().run()
    except (OSError, ValueError) as error:
        rospy.logfatal("Telemetry uploader configuration error: %s", error)
        raise


if __name__ == "__main__":
    main()
