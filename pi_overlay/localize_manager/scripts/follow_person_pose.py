#!/usr/bin/env python3
"""Fuse the selected camera track with LaserScan and publish a base-frame pose."""

import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class FollowPersonPose:
    def __init__(self):
        self.robot_code = str(rospy.get_param("~robot_code", "MEDIBOT-01"))
        self.base_frame = str(rospy.get_param("~base_frame", "base_link")).lstrip("/")
        self.candidate_topic = str(rospy.get_param("~candidate_topic", "/vision/person_candidates"))
        self.follow_state_topic = str(rospy.get_param("~follow_state_topic", "/vision/follow_state"))
        self.pose_topic = str(rospy.get_param("~pose_topic", "/person_tracker/pose"))
        fov_degrees = min(150.0, max(20.0, float(rospy.get_param("~camera_horizontal_fov_degrees", 74.0))))
        self.fov_half_tangent = math.tan(math.radians(fov_degrees) / 2.0)
        self.camera_yaw_offset = math.radians(float(rospy.get_param("~camera_yaw_offset_degrees", 0.0)))
        self.max_target_bearing = math.radians(min(85.0, max(10.0, float(rospy.get_param("~max_target_bearing_degrees", 65.0)))))
        self.max_candidate_age = max(0.3, float(rospy.get_param("~max_candidate_age_seconds", 1.0)))
        self.lock = threading.RLock()
        self.selected_track = ""
        self.candidates = {}
        self.standoff_width_ratio = float(rospy.get_param("~standoff_width_ratio", 0.90))
        self.image_width = 0
        self.candidates_at = 0.0
        self.candidate_stamp = rospy.Time(0)
        self.last_target = None
        self.last_target_track = ""

        self.publisher = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=1)
        self.candidate_sub = rospy.Subscriber(self.candidate_topic, String, self._candidates, queue_size=1)
        self.state_sub = rospy.Subscriber(self.follow_state_topic, String, self._state, queue_size=5)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.loginfo(
            "Follow target pose ready: camera bbox -> %s; camera FOV %.1f deg, yaw offset %.1f deg",
            self.pose_topic,
            fov_degrees,
            math.degrees(self.camera_yaw_offset),
        )

    def _state(self, message):
        try:
            event = json.loads(message.data)
            status = str(event.get("status", "")).upper()
            track_id = str(event.get("trackId", ""))
            with self.lock:
                if status == "TARGET_SELECTED" and track_id:
                    if self.selected_track != track_id:
                        self.last_target = None
                        self.last_target_track = track_id
                    self.selected_track = track_id
                elif status in ("OFF", "TARGET_LOST", "WAITING_FOR_PERSON"):
                    self.selected_track = ""
                    self.last_target = None
                    self.last_target_track = ""
        except (ValueError, TypeError, AttributeError):
            rospy.logwarn_throttle(10.0, "Ignoring invalid Follow Me state")

    def _candidates(self, message):
        try:
            event = json.loads(message.data)
            if event.get("robotCode") != self.robot_code:
                return
            raw = event.get("candidates", [])
            if not isinstance(raw, list):
                return
            indexed = {}
            for candidate in raw[:3]:
                if not isinstance(candidate, dict):
                    continue
                track_id = str(candidate.get("trackId", ""))
                bbox = candidate.get("bbox")
                if track_id and isinstance(bbox, dict):
                    indexed[track_id] = dict(candidate)
            width = int(event.get("imageWidth", 0))
            with self.lock:
                self.candidates = indexed
                self.image_width = width
                self.candidates_at = time.monotonic()
                self.candidate_stamp = rospy.Time.now()
        except (ValueError, TypeError, AttributeError):
            rospy.logwarn_throttle(10.0, "Ignoring invalid Follow Me candidate frame")

    def _estimate(self, candidate, image_width):
        bbox = candidate["bbox"]
        try:
            center_x = float(bbox["x"]) + float(bbox["width"]) / 2.0
            width_ratio = float(bbox["width"]) / image_width
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            return None
        if image_width <= 1 or not math.isfinite(center_x) or not 0 < width_ratio <= 1:
            return None
        normalized_x = 1.0 - 2.0 * center_x / image_width
        # Ignore detector jitter in the central 12% of the frame.
        deadband = 0.12
        normalized_x = math.copysign(
            max(0.0, abs(normalized_x) - deadband) / (1.0 - deadband),
            normalized_x,
        )
        bearing = self.camera_yaw_offset + math.atan(normalized_x * self.fov_half_tangent)
        if abs(bearing) > self.max_target_bearing:
            return None
        # At the configured standoff, the box should cover this fraction of the image.
        distance = min(1.35, max(0.75, 1.0 + 2.5 * (self.standoff_width_ratio - width_ratio)))
        return distance * math.cos(bearing), distance * math.sin(bearing)

    def _tick(self, _event):
        now = time.monotonic()
        with self.lock:
            selected = self.selected_track
            candidate = self.candidates.get(selected)
            image_width = self.image_width
            candidate_age = now - self.candidates_at
            candidate_stamp = self.candidate_stamp
        if not selected or candidate is None:
            return
        if candidate_age > self.max_candidate_age:
            return
        point = self._estimate(candidate, image_width)
        if point is None:
            return

        x, y = point
        with self.lock:
            if selected != self.selected_track:
                return
            if self.last_target is not None and self.last_target_track == selected:
                old_x, old_y = self.last_target
                if math.hypot(x - old_x, y - old_y) > 1.25:
                    rospy.logwarn_throttle(2.0, "Ignoring a discontinuous Follow Me range estimate")
                    return
                x = old_x * 0.3 + x * 0.7
                if y != 0.0 and old_y * y > 0.0:
                    y = old_y * 0.3 + y * 0.7
            self.last_target = (x, y)
            self.last_target_track = selected

        message = PoseStamped()
        message.header.stamp = candidate_stamp
        message.header.frame_id = self.base_frame
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = 0.0
        angle = math.atan2(y, x)
        message.pose.orientation.z = math.sin(angle / 2.0)
        message.pose.orientation.w = math.cos(angle / 2.0)
        self.publisher.publish(message)


def main():
    rospy.init_node("follow_person_pose", anonymous=False)
    FollowPersonPose()
    rospy.spin()


if __name__ == "__main__":
    main()
