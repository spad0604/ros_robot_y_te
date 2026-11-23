#!/usr/bin/env python3
"""Forward safety alerts from ROS to the robot alert API."""

import json
import queue
import threading
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import rospy
from std_msgs.msg import String


DEFAULT_API_BASE_URL = "https://34.172.236.168:8081/api/v1"
ALLOWED_SEVERITIES = {"INFO", "WARNING", "CRITICAL"}


class RobotAlertUploader:
    def __init__(self):
        self.api_base = str(rospy.get_param("~api_base_url", DEFAULT_API_BASE_URL)).rstrip("/")
        self.robot_code = str(rospy.get_param("~robot_code", "MEDIBOT-01")).strip()
        self.alert_topic = str(rospy.get_param("~alert_topic", "/robot/alerts"))
        self.timeout = max(2.0, float(rospy.get_param("~request_timeout_seconds", 8.0)))
        self.retry_delay = max(1.0, float(rospy.get_param("~retry_delay_seconds", 5.0)))
        if not self.api_base.startswith(("https://", "http://")):
            raise ValueError("~api_base_url must be an HTTP(S) URL")
        if not self.robot_code:
            raise ValueError("~robot_code must not be empty")

        self._pending = queue.Queue(maxsize=50)
        self._shutdown = threading.Event()
        self._subscriber = rospy.Subscriber(
            self.alert_topic, String, self._on_alert, queue_size=50
        )
        rospy.on_shutdown(self._shutdown.set)
        threading.Thread(
            target=self._upload_loop, name="robot-alert-uploader", daemon=True
        ).start()
        rospy.loginfo(
            "Robot alert uploader ready: %s -> %s/robots/%s/alerts",
            self.alert_topic,
            self.api_base,
            quote(self.robot_code, safe=""),
        )

    def _on_alert(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("alert must be a JSON object")
            severity = str(payload.get("severity", "")).strip().upper()
            event_type = str(payload.get("type", "")).strip().upper()
            detail = str(payload.get("message", "")).strip()
            source_event_id = str(payload.get("sourceEventId", "")).strip()
            if severity not in ALLOWED_SEVERITIES:
                raise ValueError("unsupported alert severity")
            if not event_type or len(event_type) > 120:
                raise ValueError("alert type is missing or too long")
            if not detail or len(detail) > 500:
                raise ValueError("alert message is missing or too long")
            if len(source_event_id) > 128:
                raise ValueError("sourceEventId is too long")
            image_base64 = payload.get("imageBase64")
            image_content_type = payload.get("imageContentType")
            if image_base64 is not None:
                if not isinstance(image_base64, str) or len(image_base64) > 420_000:
                    raise ValueError("imageBase64 is invalid or too large")
                if image_content_type not in (None, "image/jpeg"):
                    raise ValueError("only JPEG alert images are supported")
            self._pending.put_nowait(
                {
                    "severity": severity,
                    "type": event_type,
                    "message": detail,
                    "sourceEventId": source_event_id or None,
                    **({
                        "imageBase64": image_base64,
                        "imageContentType": image_content_type or "image/jpeg",
                    } if image_base64 is not None else {}),
                }
            )
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            rospy.logerr("Discarding invalid robot alert: %s", error)
        except queue.Full:
            rospy.logerr("Robot alert queue is full; could not enqueue alert")

    def _post(self, event):
        url = "%s/robots/%s/alerts" % (
            self.api_base,
            quote(self.robot_code, safe=""),
        )
        body = json.dumps(event, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        request = Request(
            url,
            data=body,
            headers={"Accept": "application/json", "Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=self.timeout) as response:
            response.read()
            if response.status < 200 or response.status >= 300:
                raise RuntimeError("alert API returned HTTP %s" % response.status)

    def _upload_loop(self):
        while not rospy.is_shutdown() and not self._shutdown.is_set():
            try:
                event = self._pending.get(timeout=1.0)
            except queue.Empty:
                continue
            delay = self.retry_delay
            while not rospy.is_shutdown() and not self._shutdown.is_set():
                try:
                    self._post(event)
                    rospy.loginfo(
                        "Safety alert delivered to server: %s (%s)",
                        event["type"],
                        event.get("sourceEventId") or "no sourceEventId",
                    )
                    break
                except HTTPError as error:
                    if error.code < 500:
                        rospy.logerr(
                            "Server rejected safety alert with HTTP %s; dropping event",
                            error.code,
                        )
                        break
                    rospy.logwarn("Alert API returned HTTP %s; retrying", error.code)
                except (URLError, OSError, RuntimeError) as error:
                    rospy.logwarn("Could not upload safety alert (%s); retrying", error)
                if self._shutdown.wait(delay):
                    break
                delay = min(delay * 2.0, 60.0)
            self._pending.task_done()


def main():
    rospy.init_node("robot_alert_uploader", anonymous=False)
    RobotAlertUploader()
    rospy.spin()


if __name__ == "__main__":
    main()
