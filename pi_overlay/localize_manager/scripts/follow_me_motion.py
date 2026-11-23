#!/usr/bin/env python3
"""Run Follow Me through moving_server; never publish velocity directly."""

import json
import os
import re
import threading
import time
import uuid

import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from moving_server_msgs.msg import (
    MoveCommandActionGoal,
    MoveCommandActionResult,
    MoveCommandGoal,
    MovingServerState,
)
from localize_manager.srv import StartLocalization
from std_msgs.msg import String


ACTIVE_GOAL_STATES = {
    GoalStatus.PENDING,
    GoalStatus.ACTIVE,
    GoalStatus.PREEMPTING,
    GoalStatus.RECALLING,
}


class FollowMeMotion:
    def __init__(self):
        self.lock = threading.RLock()
        self.pose_topic = rospy.get_param("~pose_topic", "/person_tracker/pose")
        self.action_name = rospy.get_param(
            "~action_name", "/moving_server/move_command"
        ).rstrip("/")
        self.follow_distance = max(
            0.4, float(rospy.get_param("~follow_distance", 1.0))
        )
        self.goal_timeout = max(
            10.0, float(rospy.get_param("~goal_timeout_seconds", 3600.0))
        )
        self.pose_timeout = max(
            0.5, float(rospy.get_param("~pose_timeout_seconds", 1.0))
        )
        self.start_timeout = max(
            1.0, float(rospy.get_param("~start_timeout_seconds", 3.0))
        )
        self.localization_timeout = max(
            5.0, float(rospy.get_param("~localization_timeout_seconds", 45.0))
        )
        self.localization_service = rospy.get_param(
            "~start_localization_service", "/localize_manager/start_localization"
        )
        self.active_map_name_param = rospy.get_param(
            "~active_map_name_param", "/localize_manager/active_map_name"
        )
        self.goal_journal = os.path.expanduser(
            rospy.get_param("~goal_journal_file", "~/.ros/follow_me_goal.json")
        )

        self.enabled = False
        self.detector_status = "OFF"
        self.selected_track = ""
        self.label = ""
        self.requested_track = ""
        self.requested_at = 0.0
        self.latest_pose_at = 0.0
        self.moving_state = None
        self.moving_state_at = 0.0
        self.localize_state = ""
        self.localization_pending = False
        self.localization_started_at = 0.0
        self.localization_retry_at = 0.0
        self.localization_error = ""
        self.localization_generation = 0
        self.action_status = []
        self.action_status_at = 0.0
        self.goal_active = False
        self.goal_track = ""
        self.goal_id = ""
        self.cancel_requested = False
        self.cancel_status = None
        self.next_cancel_at = 0.0
        self.recovering_goal_id = self._read_goal_journal()
        self.recovery_missing_since = 0.0
        self.status = "OFF"

        self.state_pub = rospy.Publisher(
            "/vision/follow_motion_state", String, queue_size=1, latch=True
        )
        self.goal_pub = rospy.Publisher(
            self.action_name + "/goal", MoveCommandActionGoal, queue_size=1
        )
        self.cancel_pub = rospy.Publisher(
            self.action_name + "/cancel", GoalID, queue_size=1
        )
        rospy.Subscriber(
            "/vision/follow_command", String, self._command, queue_size=10
        )
        rospy.Subscriber(
            "/vision/follow_state", String, self._detector_state, queue_size=5
        )
        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose, queue_size=1)
        self._state_sub = rospy.Subscriber(
            rospy.get_param("~moving_state_topic", "/moving_server/state"),
            MovingServerState,
            self._moving_state,
            queue_size=1,
        )
        self._status_sub = rospy.Subscriber(
            self.action_name + "/status", GoalStatusArray, self._action_status,
            queue_size=1,
        )
        self._result_sub = rospy.Subscriber(
            self.action_name + "/result",
            MoveCommandActionResult,
            self._result,
            queue_size=1,
        )
        self._localize_sub = rospy.Subscriber(
            rospy.get_param("~localize_state_topic", "/localize_manager/state"),
            String,
            self._localize_state,
            queue_size=1,
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)
        self._publish(
            "MOTION_RECOVERING" if self.recovering_goal_id else "OFF", force=True
        )
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "Follow Me motion ready: %s -> %s via %s",
            self.pose_topic,
            self.action_name,
            "MODE_TRACK_PERSON",
        )

    def _read_goal_journal(self):
        try:
            with open(self.goal_journal, encoding="utf-8") as source:
                goal_id = str(json.load(source).get("goalId", ""))
            return goal_id if goal_id.startswith("follow-me-") else ""
        except (OSError, ValueError, TypeError, AttributeError):
            return ""

    def _write_goal_journal(self, goal_id):
        directory = os.path.dirname(self.goal_journal) or "."
        os.makedirs(directory, exist_ok=True)
        temporary = self.goal_journal + ".tmp"
        if goal_id:
            with open(temporary, "w", encoding="utf-8") as target:
                json.dump({"goalId": goal_id}, target)
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, self.goal_journal)
        else:
            try:
                os.unlink(self.goal_journal)
            except FileNotFoundError:
                pass

    def _clear_recovered_goal(self):
        with self.lock:
            try:
                self._write_goal_journal("")
            except OSError as error:
                rospy.logerr("Could not clear recovered Follow Me goal ID: %s", error)
            self.recovering_goal_id = ""
            self.recovery_missing_since = 0.0
            self._publish("MOTION_PREEMPTED", force=True)

    def _recover_goal_if_needed(self, now):
        with self.lock:
            goal_id = self.recovering_goal_id
            statuses = list(self.action_status)
            status_at = self.action_status_at
            state = self.moving_state
            state_connected = self._state_sub.get_num_connections() > 0
            cancel_at = self.next_cancel_at
        if not goal_id:
            return False

        matching = next((item for item in statuses if item.goal_id.id == goal_id), None)
        if matching is not None and matching.status in ACTIVE_GOAL_STATES:
            if now >= cancel_at and self.cancel_pub.get_num_connections() > 0:
                self.cancel_pub.publish(GoalID(stamp=rospy.Time(0), id=goal_id))
                with self.lock:
                    self.next_cancel_at = now + 0.5
            with self.lock:
                self.recovery_missing_since = 0.0
            return True
        if matching is not None:
            self._clear_recovered_goal()
            return False

        fresh_status = status_at > 0.0 and now - status_at <= 2.0
        fresh_state = state_connected and state is not None
        if fresh_state and state.state_name != "IDLE":
            if now >= cancel_at and self.cancel_pub.get_num_connections() > 0:
                self.cancel_pub.publish(GoalID(stamp=rospy.Time(0), id=goal_id))
                with self.lock:
                    self.next_cancel_at = now + 0.5
        if fresh_status and fresh_state and state.state_name == "IDLE":
            with self.lock:
                if not self.recovery_missing_since:
                    self.recovery_missing_since = now
                missing_for = now - self.recovery_missing_since
            if missing_for >= 1.0:
                self._clear_recovered_goal()
                return False
        else:
            with self.lock:
                self.recovery_missing_since = 0.0
        return True

    def _result(self, message):
        goal_id = message.status.goal_id.id
        with self.lock:
            if not self.goal_active or goal_id != self.goal_id:
                return
            cancel_status = self.cancel_status
            try:
                self._write_goal_journal("")
            except OSError as error:
                rospy.logerr("Could not clear Follow Me goal ID: %s", error)
            self.goal_active = False
            self.goal_track = ""
            self.goal_id = ""
            self.cancel_requested = False
            self.cancel_status = None
            self.next_cancel_at = 0.0
            if cancel_status is not None:
                status = cancel_status
            elif (
                message.status.status == GoalStatus.SUCCEEDED
                and message.result.success
            ):
                status = "FOLLOW_COMPLETE"
            elif message.status.status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
                status = "MOTION_PREEMPTED"
            else:
                status = "MOTION_FAILED"
            self._publish(status, force=True)
        rospy.loginfo("Follow Me moving_server goal finished with status %s", status)

    def _publish(self, status=None, force=False):
        with self.lock:
            if status is not None:
                self.status = status
            event = {
                "enabled": self.enabled,
                "trackId": self.selected_track or self.requested_track or self.goal_track,
                "label": self.label,
                "status": self.status,
                "timestamp": int(time.time() * 1000),
            }
            key = (event["enabled"], event["trackId"], event["label"], event["status"])
            if not force and key == getattr(self, "_published_key", None):
                return
            self._published_key = key
        self.state_pub.publish(String(data=json.dumps(event, separators=(",", ":"))))

    def _command(self, message):
        try:
            event = json.loads(message.data)
            action = str(event.get("action", "")).upper()
            track_id = str(event.get("trackId", ""))
        except (ValueError, TypeError, AttributeError):
            rospy.logwarn_throttle(10.0, "Ignoring invalid Follow Me command")
            return

        if action == "ENABLE":
            self._cancel_current("WAITING_FOR_PERSON")
            with self.lock:
                self.enabled = True
                self.requested_track = ""
                self.selected_track = ""
                self.detector_status = "WAITING_FOR_PERSON"
                self.localization_error = ""
                self.localization_started_at = 0.0
            self._publish("WAITING_FOR_PERSON")
            self._ensure_localizing()
        elif action == "STOP":
            self._cancel_current("OFF")
            with self.lock:
                self.enabled = False
                self.requested_track = ""
                self.selected_track = ""
                self.label = ""
                self.detector_status = "OFF"
                self.localization_generation += 1
                self.localization_pending = False
                self.localization_error = ""
            self._publish("OFF")
        elif action == "START":
            if not track_id:
                self._publish("TARGET_LOST")
                return
            self._cancel_current(None)
            with self.lock:
                self.enabled = True
                self.requested_track = track_id
                self.requested_at = time.monotonic()
                self.status = "STARTING"
            self._publish("STARTING", force=True)
            self._ensure_localizing()

    def _selected_map_name(self):
        map_name = str(rospy.get_param(self.active_map_name_param, "")).strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", map_name):
            return ""
        return map_name

    def _ensure_localizing(self):
        now = time.monotonic()
        with self.lock:
            if not self.enabled:
                return
            if self.localization_pending or now < self.localization_retry_at:
                return
            if self.localize_state == "SAVING_MAP":
                self.localization_error = "MAP_SAVE_IN_PROGRESS"
                self.localization_retry_at = now + 3.0
                self._publish(self.localization_error, force=True)
                return

        map_name = self._selected_map_name()
        if not map_name:
            with self.lock:
                self.localization_error = "MAP_NOT_SELECTED"
                self.localization_retry_at = now + 3.0
                self._publish(self.localization_error, force=True)
            return

        with self.lock:
            if not self.enabled:
                return
            if self.localization_pending:
                return
            if self.localize_state == "LOCALIZING":
                self.localization_error = ""
                self.localization_retry_at = 0.0
                if self.status in ("MAP_NOT_SELECTED", "MAP_SAVE_IN_PROGRESS"):
                    self.status = "STARTING" if self.requested_track else "WAITING_FOR_PERSON"
                    self._publish(self.status, force=True)
                return
            self.localization_generation += 1
            generation = self.localization_generation
            self.localization_pending = True
            self.localization_started_at = now
            self.localization_error = ""
            self._publish("LOCALIZATION_STARTING", force=True)

        threading.Thread(
            target=self._start_localization,
            args=(map_name, generation),
            name="follow-me-localization",
            daemon=True,
        ).start()

    def _start_localization(self, map_name, generation):
        try:
            rospy.wait_for_service(self.localization_service, timeout=8.0)
            with self.lock:
                if generation != self.localization_generation or not self.enabled:
                    return
            response = rospy.ServiceProxy(
                self.localization_service, StartLocalization
            )(map_name)
            if not response.success:
                raise RuntimeError(str(response.message or "service rejected request"))
            rospy.loginfo("Follow Me started localization with map %s", map_name)
        except (rospy.ROSException, rospy.ServiceException, RuntimeError, OSError) as error:
            with self.lock:
                if generation != self.localization_generation or not self.enabled:
                    return
                self.localization_pending = False
                self.localization_started_at = 0.0
                self.localization_error = "LOCALIZATION_FAILED"
                self.requested_track = ""
                self._publish(self.localization_error, force=True)
            rospy.logerr("Follow Me could not start localization: %s", error)
            return

        with self.lock:
            if generation != self.localization_generation or not self.enabled:
                return
            self.localization_pending = False
            self.localization_started_at = time.monotonic()
            self.localization_error = ""
            self._publish("LOCALIZATION_STARTING", force=True)

    def _detector_state(self, message):
        try:
            event = json.loads(message.data)
            status = str(event.get("status", "")).upper()
            track_id = str(event.get("trackId", ""))
        except (ValueError, TypeError, AttributeError):
            rospy.logwarn_throttle(10.0, "Ignoring invalid detector Follow Me state")
            return

        cancel_status = None
        with self.lock:
            self.enabled = event.get("enabled") is True
            self.detector_status = status
            if self.selected_track != track_id:
                self.latest_pose_at = 0.0
            self.selected_track = track_id
            self.label = str(event.get("label", ""))
            if status in ("OFF", "WAITING_FOR_PERSON", "TARGET_LOST"):
                self.requested_track = ""
                if self.goal_active:
                    cancel_status = status
            elif status == "TARGET_SELECTED":
                if self.requested_track and self.requested_track != track_id:
                    self.requested_track = ""
                if not self.goal_active and not self.requested_track:
                    self.status = "TARGET_SELECTED"

        if cancel_status is not None:
            self._cancel_current(cancel_status)
        else:
            with self.lock:
                if self.requested_track and status == "TARGET_SELECTED":
                    status_to_publish = "STARTING"
                elif (
                    status == "WAITING_FOR_PERSON"
                    and self.enabled
                    and self.localize_state != "LOCALIZING"
                ):
                    status_to_publish = self.localization_error or (
                        "LOCALIZATION_STARTING"
                        if self.localization_pending or self.localization_started_at
                        else "WAITING_FOR_PERSON"
                    )
                else:
                    status_to_publish = status
            self._publish(status_to_publish, force=True)

    def _pose(self, message):
        age = (rospy.Time.now() - message.header.stamp).to_sec()
        if message.header.stamp == rospy.Time(0) or not 0 <= age <= self.pose_timeout:
            return
        with self.lock:
            self.latest_pose_at = time.monotonic() - age

    def _moving_state(self, message):
        with self.lock:
            self.moving_state = message
            self.moving_state_at = time.monotonic()

    def _action_status(self, message):
        with self.lock:
            self.action_status = list(message.status_list)
            self.action_status_at = time.monotonic()

    def _localize_state(self, message):
        publish_status = None
        with self.lock:
            self.localize_state = message.data.strip().upper()
            if self.localize_state == "LOCALIZING":
                self.localization_pending = False
                self.localization_started_at = 0.0
                self.localization_retry_at = 0.0
                self.localization_error = ""
                if self.enabled and self.status in (
                    "LOCALIZATION_STARTING",
                    "MAP_NOT_SELECTED",
                    "MAP_SAVE_IN_PROGRESS",
                    "LOCALIZATION_FAILED",
                ):
                    self.status = "STARTING" if self.requested_track else "WAITING_FOR_PERSON"
                    publish_status = self.status
        if publish_status:
            self._publish(publish_status, force=True)

    def _gateway_readiness(self):
        with self.lock:
            state = self.moving_state
            statuses = list(self.action_status)
            status_age = time.monotonic() - self.action_status_at
            state_connected = self._state_sub.get_num_connections() > 0
            status_connected = self._status_sub.get_num_connections() > 0
            localize_connected = self._localize_sub.get_num_connections() > 0
            localize_state = self.localize_state
            goal_connected = self.goal_pub.get_num_connections() > 0
            cancel_connected = self.cancel_pub.get_num_connections() > 0
            result_connected = self._result_sub.get_num_connections() > 0

        if (
            not goal_connected
            or not cancel_connected
            or not result_connected
            or not state_connected
            or not status_connected
            or not localize_connected
            or state is None
            or status_age > 2.0
        ):
            return "UNAVAILABLE"
        if localize_state != "LOCALIZING":
            return "MODE_UNAVAILABLE"
        if state.active or state.state_name != "IDLE":
            return "BUSY"
        if any(item.status in ACTIVE_GOAL_STATES for item in statuses):
            return "BUSY"
        return "READY"

    def _tick(self, _event):
        now = time.monotonic()
        if self._recover_goal_if_needed(now):
            return
        with self.lock:
            enabled = self.enabled
            localize_state = self.localize_state
            localization_error = self.localization_error
            localization_pending = self.localization_pending
            localization_started_at = self.localization_started_at
            localization_retry_at = self.localization_retry_at
            goal_active = self.goal_active
            goal_track = self.goal_track
            detector_status = self.detector_status
            selected_track = self.selected_track
            requested_track = self.requested_track
            requested_at = self.requested_at
            pose_age = now - self.latest_pose_at
        active_map_name = self._selected_map_name() if enabled else ""

        if enabled and not active_map_name:
            if localization_error != "MAP_NOT_SELECTED" or now >= localization_retry_at:
                self._ensure_localizing()
        elif (
            enabled
            and active_map_name
            and localize_state == "LOCALIZING"
            and localization_error in ("MAP_NOT_SELECTED", "MAP_SAVE_IN_PROGRESS")
        ):
            with self.lock:
                self.localization_error = ""
                self.localization_retry_at = 0.0
                next_status = "STARTING" if self.requested_track else "WAITING_FOR_PERSON"
                self.status = next_status
            self._publish(next_status, force=True)
        elif enabled and localize_state != "LOCALIZING":
            if localization_error in ("MAP_NOT_SELECTED", "MAP_SAVE_IN_PROGRESS"):
                if now >= localization_retry_at:
                    self._ensure_localizing()
            elif localization_started_at and now - localization_started_at > self.localization_timeout:
                with self.lock:
                    self.localization_generation += 1
                    self.localization_pending = False
                    self.localization_started_at = 0.0
                    self.localization_error = "LOCALIZATION_FAILED"
                    self.requested_track = ""
                self._publish("LOCALIZATION_FAILED", force=True)
                return

        if goal_active:
            with self.lock:
                cancel_requested = self.cancel_requested
                next_cancel_at = self.next_cancel_at
                active_goal_id = self.goal_id
            if cancel_requested:
                if now >= next_cancel_at and self.cancel_pub.get_num_connections() > 0:
                    self.cancel_pub.publish(
                        GoalID(stamp=rospy.Time(0), id=active_goal_id)
                    )
                    with self.lock:
                        self.next_cancel_at = now + 0.5
                return
            if localize_state != "LOCALIZING" or not active_map_name:
                self._cancel_current("LOCALIZATION_STARTING")
            elif detector_status != "TARGET_SELECTED" or selected_track != goal_track:
                self._cancel_current("TARGET_LOST")
            elif pose_age > self.pose_timeout:
                self._cancel_current("TARGET_RANGE_UNAVAILABLE")
            return
        if not requested_track:
            return
        if detector_status != "TARGET_SELECTED" or selected_track != requested_track:
            if now - requested_at > self.start_timeout:
                with self.lock:
                    self.requested_track = ""
                self._publish("TARGET_LOST")
            return
        if not active_map_name:
            if now - requested_at > self.localization_timeout:
                with self.lock:
                    self.requested_track = ""
                self._publish("MAP_NOT_SELECTED", force=True)
                return
            self._publish("MAP_NOT_SELECTED")
            return
        if localize_state != "LOCALIZING":
            if localization_error == "LOCALIZATION_FAILED":
                with self.lock:
                    self.requested_track = ""
                self._publish("LOCALIZATION_FAILED", force=True)
                return
            if localization_error in ("MAP_NOT_SELECTED", "MAP_SAVE_IN_PROGRESS"):
                if now - requested_at > self.localization_timeout:
                    with self.lock:
                        self.requested_track = ""
                    self._publish(localization_error, force=True)
                    return
                self._publish(localization_error)
                return
            if localization_pending or localization_started_at:
                self._publish("LOCALIZATION_STARTING")
                return
            self._ensure_localizing()
            self._publish("LOCALIZATION_STARTING")
            return
        if pose_age > self.pose_timeout:
            if now - requested_at > self.start_timeout:
                with self.lock:
                    self.requested_track = ""
                self._publish("TARGET_RANGE_UNAVAILABLE")
                return
            self._publish("STARTING")
            return

        readiness = self._gateway_readiness()
        if readiness == "UNAVAILABLE":
            if now - requested_at >= self.start_timeout:
                with self.lock:
                    self.requested_track = ""
                self._publish("MOTION_UNAVAILABLE")
            return
        if readiness == "MODE_UNAVAILABLE":
            with self.lock:
                self.requested_track = ""
            self._publish("LOCALIZATION_NOT_ACTIVE")
            return
        if readiness == "BUSY":
            with self.lock:
                self.requested_track = ""
            self._publish("MOTION_BUSY")
            return

        message = MoveCommandActionGoal()
        message.header.stamp = rospy.Time.now()
        message.goal_id.stamp = message.header.stamp
        message.goal_id.id = "follow-me-" + uuid.uuid4().hex
        message.goal.mode = MoveCommandGoal.MODE_TRACK_PERSON
        message.goal.target_topic = self.pose_topic
        message.goal.follow_distance = self.follow_distance
        message.goal.hold_time = 0.0
        message.goal.timeout = self.goal_timeout
        with self.lock:
            # Keep command callbacks outside the goal journal/publish window;
            # STOP or target-loss then cancels only after this exact ID exists.
            if (
                self.requested_track != requested_track
                or self.detector_status != "TARGET_SELECTED"
                or self.selected_track != requested_track
                or self.goal_active
            ):
                return
            self.goal_active = True
            self.goal_track = requested_track
            self.goal_id = message.goal_id.id
            self.cancel_requested = False
            self.cancel_status = None
            self.next_cancel_at = 0.0
            self.requested_track = ""
            try:
                self._write_goal_journal(message.goal_id.id)
            except OSError as error:
                self.goal_active = False
                self.goal_track = ""
                self.goal_id = ""
                self._publish("MOTION_FAILED", force=True)
                rospy.logerr("Could not persist Follow Me goal ID: %s", error)
                return
            self.goal_pub.publish(message)
            self._publish("FOLLOWING", force=True)
        rospy.loginfo("Following selected person track %s", requested_track)

    def _cancel_current(self, status):
        with self.lock:
            active = self.goal_active
            if active:
                self.cancel_requested = True
                self.cancel_status = status
                self.next_cancel_at = time.monotonic() + 0.5
                goal_id = self.goal_id
            if status is not None:
                self.status = status
            if active:
                self.cancel_pub.publish(GoalID(stamp=rospy.Time(0), id=goal_id))
                rospy.loginfo("Cancelled Follow Me moving_server goal: %s", status)
            if status is not None:
                self._publish(status, force=True)

    def _shutdown(self):
        with self.lock:
            goal_id = self.goal_id if self.goal_active else self.recovering_goal_id
        if goal_id and self.cancel_pub.get_num_connections() > 0:
            self.cancel_pub.publish(GoalID(stamp=rospy.Time(0), id=goal_id))
        self._cancel_current("OFF")


def main():
    rospy.init_node("follow_me_motion", anonymous=False)
    FollowMeMotion()
    rospy.spin()


if __name__ == "__main__":
    main()
