#!/usr/bin/env python3
"""Run the fire/smoke and human-fall models on one shared latest-frame ring.

The camera subscriber only replaces the one buffered frame. A single worker
then feeds that same frame to both models, so slow inference never builds an
unbounded image queue. Detection events are published on /robot/alerts for the
existing robot alert uploader.
"""

import json
import os
import threading
import time
import uuid
from collections import deque

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _normalise_label(value):
    return " ".join(str(value).strip().lower().replace("_", " ").replace("-", " ").split())


def _name_for(names, class_id):
    if isinstance(names, dict):
        return names.get(class_id, names.get(str(class_id), str(class_id)))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return names[class_id]
    return str(class_id)


class LatestFrameRing:
    """A one-slot ring: overwrite stale work and keep only the newest image."""

    def __init__(self):
        self._frames = deque(maxlen=1)
        self._sequence = 0
        self._condition = threading.Condition()

    def push(self, message):
        with self._condition:
            self._sequence += 1
            self._frames.append((self._sequence, message, time.monotonic()))
            self._condition.notify_all()

    def wait_for_newer(self, sequence, timeout):
        with self._condition:
            if self._frames and self._frames[-1][0] > sequence:
                return self._frames[-1]
            self._condition.wait(timeout=timeout)
            if self._frames and self._frames[-1][0] > sequence:
                return self._frames[-1]
            return None

    def latest(self):
        with self._condition:
            return self._frames[-1] if self._frames else None


class PersistentDetectionTrack:
    """Keep one IoU-matched detection ID and confirm it by elapsed time."""

    def __init__(self, kind, confirmation_seconds, max_gap_seconds, iou_threshold):
        self.kind = kind
        self.confirmation_seconds = confirmation_seconds
        self.max_gap_seconds = max_gap_seconds
        self.iou_threshold = iou_threshold
        self.track_id = None
        self.started_at = None
        self.last_seen = None
        self.detection = None
        self.alerted = False

    @staticmethod
    def _iou(left, right):
        lx1, ly1, lx2, ly2 = left
        rx1, ry1, rx2, ry2 = right
        x1, y1 = max(lx1, rx1), max(ly1, ry1)
        x2, y2 = min(lx2, rx2), min(ly2, ry2)
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        left_area = max(0, lx2 - lx1) * max(0, ly2 - ly1)
        right_area = max(0, rx2 - rx1) * max(0, ry2 - ry1)
        union = left_area + right_area - intersection
        return intersection / union if union else 0.0

    def _reset(self):
        self.track_id = None
        self.started_at = None
        self.last_seen = None
        self.detection = None
        self.alerted = False

    def update(self, detection, now):
        if detection is None:
            if self.last_seen is not None and now - self.last_seen > self.max_gap_seconds:
                self._reset()
            return {
                "track_id": self.track_id,
                "detection": None,
                "elapsed": max(0.0, now - self.started_at) if self.started_at else 0.0,
                "confirmed": False,
                "newly_confirmed": False,
            }

        box = detection[2]
        matched = (
            self.track_id is not None
            and self.last_seen is not None
            and now - self.last_seen <= self.max_gap_seconds
            and self._iou(self.detection[2], box) >= self.iou_threshold
        )
        if not matched:
            self._reset()
            self.track_id = "%s-%s" % (self.kind, uuid.uuid4().hex[:8])
            self.started_at = now

        self.last_seen = now
        self.detection = detection
        elapsed = now - self.started_at
        confirmed = elapsed >= self.confirmation_seconds
        newly_confirmed = confirmed and not self.alerted
        if newly_confirmed:
            self.alerted = True
        return {
            "track_id": self.track_id,
            "detection": detection,
            "elapsed": elapsed,
            "confirmed": confirmed,
            "newly_confirmed": newly_confirmed,
        }


class VisionAlertDetector:
    def __init__(self):
        from ultralytics import YOLO

        self.image_topic = str(rospy.get_param("~image_topic", "/camera/image_raw"))
        self.camera_device = str(rospy.get_param("~camera_device", ""))
        self.camera_width = int(rospy.get_param("~camera_width", 640))
        self.camera_height = int(rospy.get_param("~camera_height", 480))
        self.camera_fps = max(1.0, float(rospy.get_param("~camera_fps", 10.0)))
        self.alert_topic = str(rospy.get_param("~alert_topic", "/robot/alerts"))
        self.robot_code = str(rospy.get_param("~robot_code", "MEDIBOT-01"))
        self.fire_model_path = os.path.expanduser(str(rospy.get_param("~fire_model", "")))
        self.fall_model_path = os.path.expanduser(str(rospy.get_param("~fall_model", "")))
        self.confidence = float(rospy.get_param("~confidence", 0.55))
        self.fire_confidence = float(rospy.get_param("~fire_confidence", self.confidence))
        self.fall_confidence = float(rospy.get_param("~fall_confidence", self.confidence))
        self.image_size = int(rospy.get_param("~image_size", 640))
        self.device = str(rospy.get_param("~device", "cpu"))
        self.cpu_threads = max(1, int(rospy.get_param("~cpu_threads", 2)))
        self.target_fps = max(0.1, float(rospy.get_param("~target_fps", 2.0)))
        self.max_frame_age = max(0.5, float(rospy.get_param("~max_frame_age_seconds", 2.5)))
        confirmation_seconds = max(
            0.5, float(rospy.get_param("~confirmation_duration_seconds", 3.0))
        )
        track_max_gap = max(
            0.25, float(rospy.get_param("~track_max_gap_seconds", 1.25))
        )
        track_iou = min(
            1.0, max(0.0, float(rospy.get_param("~track_iou_threshold", 0.15)))
        )
        self._shutdown = threading.Event()
        self._ring = LatestFrameRing()
        self._fire_track = PersistentDetectionTrack(
            "FIRE", confirmation_seconds, track_max_gap, track_iou
        )
        self._fall_track = PersistentDetectionTrack(
            "FALL", confirmation_seconds, track_max_gap, track_iou
        )
        self.confirmation_seconds = confirmation_seconds
        self._alert_pub = rospy.Publisher(self.alert_topic, String, queue_size=10)
        self._preview_window_name = "MediBot safety detection"
        self._preview_window_open = False

        for path in (self.fire_model_path, self.fall_model_path):
            if not path or not os.path.isfile(path):
                raise RuntimeError("Model file does not exist: %s" % (path or "<unset>"))

        try:
            import torch
            torch.set_num_threads(self.cpu_threads)
            torch.set_num_interop_threads(1)
        except (ImportError, RuntimeError):
            pass

        rospy.loginfo("Loading fire/smoke model: %s", self.fire_model_path)
        self._fire_model = YOLO(self.fire_model_path)
        rospy.loginfo("Loading fall model: %s", self.fall_model_path)
        self._fall_model = YOLO(self.fall_model_path)

        self._subscriber = rospy.Subscriber(
            self.image_topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24
        )
        rospy.on_shutdown(self._shutdown.set)
        if self.camera_device:
            self._camera_worker = threading.Thread(
                target=self._camera_loop, name="vision-camera-capture", daemon=True
            )
            self._camera_worker.start()
        self._worker = threading.Thread(
            target=self._inference_loop, name="vision-alert-inference", daemon=True
        )
        self._worker.start()
        rospy.loginfo(
            "Vision alert detector ready: camera=%s topic=%s -> shared %d FPS frame ring -> two models -> %s",
            self.camera_device or "ROS topic",
            self.image_topic,
            int(round(self.target_fps)),
            self.alert_topic,
        )

    def _on_image(self, message):
        self._ring.push(message)

    def _camera_loop(self):
        while not rospy.is_shutdown() and not self._shutdown.is_set():
            capture = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                rospy.logerr_throttle(
                    30, "Could not open camera device %s", self.camera_device
                )
                self._shutdown.wait(3.0)
                continue

            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            capture.set(cv2.CAP_PROP_FPS, self.camera_fps)
            rospy.loginfo(
                "Camera opened on %s at requested %dx%d / %.1f FPS",
                self.camera_device,
                self.camera_width,
                self.camera_height,
                self.camera_fps,
            )
            while not rospy.is_shutdown() and not self._shutdown.is_set():
                ok, frame = capture.read()
                if ok and frame is not None:
                    self._ring.push(frame)
                else:
                    rospy.logwarn_throttle(
                        30, "Camera read failed on %s; reopening", self.camera_device
                    )
                    break
            capture.release()
            self._shutdown.wait(1.0)

    def _image_to_bgr8(self, message):
        encoding = str(message.encoding).lower()
        formats = {
            "bgr8": (3, None),
            "rgb8": (3, cv2.COLOR_RGB2BGR),
            "8uc3": (3, None),
            "bgra8": (4, cv2.COLOR_BGRA2BGR),
            "rgba8": (4, cv2.COLOR_RGBA2BGR),
            "mono8": (1, cv2.COLOR_GRAY2BGR),
            "8uc1": (1, cv2.COLOR_GRAY2BGR),
        }
        if encoding not in formats:
            raise ValueError("Unsupported camera encoding: %s" % message.encoding)
        channels, conversion = formats[encoding]
        row_bytes = int(message.width) * channels
        step = int(message.step)
        if message.height <= 0 or message.width <= 0 or step < row_bytes:
            raise ValueError("Invalid image dimensions or row step")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        required = int(message.height) * step
        if raw.size < required:
            raise ValueError("Image data is shorter than height * step")
        rows = raw[:required].reshape(int(message.height), step)
        packed = rows[:, :row_bytes]
        image = packed.reshape(int(message.height), int(message.width), channels)
        if channels == 1:
            image = image.reshape(int(message.height), int(message.width))
        if conversion is not None:
            image = cv2.cvtColor(image, conversion)
        return np.ascontiguousarray(image)

    def _detections(self, model, image, accepted_labels, confidence):
        results = model.predict(
            source=image,
            imgsz=self.image_size,
            conf=confidence,
            max_det=30,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        names = getattr(result, "names", getattr(model, "names", {}))
        detections = []
        for box in boxes:
            class_id = int(box.cls.item())
            label = str(_name_for(names, class_id))
            normalized = _normalise_label(label)
            if normalized not in accepted_labels:
                continue
            score = float(box.conf.item())
            coordinates = tuple(int(value) for value in box.xyxy[0].tolist())
            detections.append((label, score, coordinates))
        return detections

    @staticmethod
    def _best_detection(detections):
        return max(detections, key=lambda detection: detection[1]) if detections else None

    def _update_preview(
        self, image, fire, fall, standing, sitting, fire_track, fall_track
    ):
        show_window = bool(rospy.get_param("~show_window", False))
        if not show_window:
            if self._preview_window_open:
                try:
                    cv2.destroyWindow(self._preview_window_name)
                except cv2.error:
                    pass
                self._preview_window_open = False
            return
        if not os.environ.get("DISPLAY"):
            rospy.logerr_throttle(30, "VNC preview requested but DISPLAY is not set")
            return

        try:
            preview = image.copy()
            height, width = preview.shape[:2]
            if fire:
                status = "FIRE/SMOKE %s %s %.1f/%.1fs" % (
                    fire[0],
                    fire_track["track_id"],
                    fire_track["elapsed"],
                    self.confirmation_seconds,
                )
            else:
                status = "FIRE/SMOKE: clear"
            cv2.putText(
                preview,
                status,
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255)
                if fire_track["confirmed"]
                else ((0, 165, 255) if fire else (0, 200, 0)),
                2,
                cv2.LINE_AA,
            )
            if fall:
                fall_status = "FALL %s %s %.1f/%.1fs" % (
                    fall[0],
                    fall_track["track_id"],
                    fall_track["elapsed"],
                    self.confirmation_seconds,
                )
            elif standing:
                fall_status = "PERSON standing %.0f%%" % (standing[1] * 100.0)
            elif sitting:
                fall_status = "PERSON sitting %.0f%%" % (sitting[1] * 100.0)
            else:
                fall_status = "PERSON: none"
            cv2.putText(
                preview,
                fall_status,
                (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255)
                if fall_track["confirmed"]
                else ((0, 165, 255) if fall else (0, 200, 0)),
                2,
                cv2.LINE_AA,
            )
            for detection, color, track_id in (
                (fire, (0, 0, 255), fire_track["track_id"]),
                (fall, (255, 80, 0), fall_track["track_id"]),
                (standing, (0, 200, 0), None),
                (sitting, (255, 200, 0), None),
            ):
                if detection is None:
                    continue
                label, score, coordinates = detection
                x1, y1, x2, y2 = coordinates
                cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)
                caption = "%s %.0f%%" % (label, score * 100.0)
                if track_id:
                    caption = "%s %s" % (caption, track_id)
                cv2.putText(
                    preview,
                    caption,
                    (max(0, x1), max(78, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    color,
                    2,
                    cv2.LINE_AA,
                )
            cv2.putText(
                preview,
                "%dx%d | %.1f inference FPS" % (width, height, self.target_fps),
                (12, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            if not self._preview_window_open:
                cv2.namedWindow(self._preview_window_name, cv2.WINDOW_NORMAL)
                self._preview_window_open = True
            cv2.imshow(self._preview_window_name, preview)
            cv2.waitKey(1)
        except cv2.error as error:
            self._preview_window_open = False
            rospy.logerr_throttle(30, "Could not display VNC preview: %s", error)

    def _emit(self, event_type, detection, track_id):
        label, confidence = detection[:2]
        if event_type == "FIRE":
            detail = "khói" if _normalise_label(label) == "smoke" else "lửa"
            message = (
                "Robot phát hiện %s ổn định %.1f giây "
                "(track %s, độ tin cậy %.0f%%)."
                % (
                    detail,
                    self.confirmation_seconds,
                    track_id,
                    confidence * 100.0,
                )
            )
        else:
            message = (
                "Robot phát hiện người bị ngã ổn định %.1f giây "
                "(track %s, độ tin cậy %.0f%%)."
                % (
                    self.confirmation_seconds,
                    track_id,
                    confidence * 100.0,
                )
            )

        payload = {
            "severity": "CRITICAL",
            "type": event_type,
            "message": message,
            "sourceEventId": "%s:%s:%s:%s"
            % (self.robot_code, event_type, track_id, uuid.uuid4().hex),
        }
        self._alert_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        rospy.logwarn("Safety alert published: %s", json.dumps(payload, ensure_ascii=False))

    def _inference_loop(self):
        min_interval = 1.0 / self.target_fps
        last_sequence = 0
        last_started = 0.0
        while not rospy.is_shutdown() and not self._shutdown.is_set():
            frame = self._ring.wait_for_newer(last_sequence, timeout=1.0)
            if frame is None:
                rospy.logwarn_throttle(
                    30, "No camera frames received yet on %s", self.image_topic
                )
                continue

            remaining = min_interval - (time.monotonic() - last_started)
            if last_started and remaining > 0 and self._shutdown.wait(remaining):
                return

            frame = self._ring.latest()
            if frame is None:
                continue
            sequence, message, received_at = frame
            if sequence <= last_sequence:
                continue
            last_sequence = sequence
            last_started = time.monotonic()
            if last_started - received_at > self.max_frame_age:
                rospy.logwarn_throttle(30, "Skipping stale camera frames on %s", self.image_topic)
                continue

            try:
                image = (
                    np.ascontiguousarray(message)
                    if isinstance(message, np.ndarray)
                    else self._image_to_bgr8(message)
                )
            except Exception as error:
                rospy.logwarn_throttle(30, "Could not decode camera frame: %s", error)
                continue

            fire = None
            fall = None
            standing = None
            sitting = None
            fire_inference_ok = True
            fall_inference_ok = True
            try:
                fire = self._best_detection(
                    self._detections(
                        self._fire_model,
                        image,
                        {"fire", "smoke", "flame"},
                        self.fire_confidence,
                    )
                )
            except Exception as error:
                fire_inference_ok = False
                rospy.logerr_throttle(30, "Fire/smoke inference failed: %s", error)
            try:
                posture_detections = self._detections(
                    self._fall_model,
                    image,
                    {
                        "fallen",
                        "fall",
                        "person fall",
                        "human fall",
                        "falling person",
                        "standing",
                        "person standing",
                        "standing person",
                        "sitting",
                        "person sitting",
                        "sitting person",
                    },
                    self.fall_confidence,
                )
                fall_labels = {
                    "fallen", "fall", "person fall", "human fall", "falling person"
                }
                standing_labels = {"standing", "person standing", "standing person"}
                sitting_labels = {"sitting", "person sitting", "sitting person"}
                fall = self._best_detection(
                    [d for d in posture_detections if _normalise_label(d[0]) in fall_labels]
                )
                standing = self._best_detection(
                    [d for d in posture_detections if _normalise_label(d[0]) in standing_labels]
                )
                sitting = self._best_detection(
                    [d for d in posture_detections if _normalise_label(d[0]) in sitting_labels]
                )
            except Exception as error:
                fall_inference_ok = False
                rospy.logerr_throttle(30, "Fall inference failed: %s", error)

            processed_at = time.monotonic()
            fire_track = self._fire_track.update(
                fire if fire_inference_ok else None, processed_at
            )
            fall_track = self._fall_track.update(
                fall if fall_inference_ok else None, processed_at
            )
            self._update_preview(
                image,
                fire,
                fall,
                standing,
                sitting,
                fire_track,
                fall_track,
            )

            if fire_track["newly_confirmed"] and fire is not None:
                self._emit("FIRE", fire, fire_track["track_id"])
            if fall_track["newly_confirmed"] and fall is not None:
                self._emit("PERSON_FALL", fall, fall_track["track_id"])
            rospy.loginfo_throttle(
                30,
                "Processed a shared camera frame with both safety models",
            )


def main():
    rospy.init_node("vision_alert_detector", anonymous=False)
    try:
        VisionAlertDetector()
        rospy.spin()
    except (OSError, RuntimeError, ValueError) as error:
        rospy.logfatal("Vision alert detector could not start: %s", error)
        raise


if __name__ == "__main__":
    main()
