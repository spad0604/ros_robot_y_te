#!/usr/bin/env python3
"""Run the fire/smoke and human-fall models on one shared latest-frame ring.

The camera subscriber only replaces the one buffered frame. A single worker
then feeds that same frame to both models concurrently, so slow inference never builds an
unbounded image queue. Detection events are published on /robot/alerts for the
existing robot alert uploader.
"""

import json
import math
import os
import base64
import threading
import time
import uuid
from collections import deque
from concurrent.futures import ThreadPoolExecutor

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


class PersonDetectionTracks:
    """Greedy IoU tracker for posture-model people, including sitting people."""

    def __init__(self, max_gap_seconds, iou_threshold):
        self.max_gap_seconds = max_gap_seconds
        self.iou_threshold = iou_threshold
        self.tracks = {}

    @staticmethod
    def _iou(left, right):
        return PersistentDetectionTrack._iou(left, right)

    def update(self, detections, now):
        for track_id in list(self.tracks):
            if now - self.tracks[track_id]["last_seen"] > self.max_gap_seconds:
                del self.tracks[track_id]

        available = set(self.tracks)
        result = []
        for detection in sorted(detections, key=lambda item: item[1], reverse=True):
            box = detection[2]
            match_id, match_iou = None, self.iou_threshold
            for track_id in available:
                record = self.tracks[track_id]
                overlap = max(
                    self._iou(record["box"], box),
                    self._iou(self._predicted_box(record, now), box),
                )
                if overlap >= match_iou:
                    match_id, match_iou = track_id, overlap
            if match_id is None and len(detections) == 1 and len(available) == 1:
                only_id = next(iter(available))
                if self._nearby(self.tracks[only_id]["box"], box):
                    match_id = only_id
            if match_id is None:
                match_id = "person-%s" % uuid.uuid4().hex[:8]
            else:
                available.remove(match_id)
            previous = self.tracks.get(match_id)
            velocity = (0.0, 0.0)
            if previous and now > previous["last_seen"]:
                dt = now - previous["last_seen"]
                old = previous["box"]
                velocity = (
                    (box[0] + box[2] - old[0] - old[2]) / (2 * dt),
                    (box[1] + box[3] - old[1] - old[3]) / (2 * dt),
                )
            self.tracks[match_id] = {
                "box": box,
                "velocity": velocity,
                "detection": detection,
                "last_seen": now,
            }
            result.append({"track_id": match_id, "detection": detection})
        return result

    @staticmethod
    def _predicted_box(record, now):
        x1, y1, x2, y2 = record["box"]
        vx, vy = record["velocity"]
        elapsed = min(1.0, now - record["last_seen"])
        limit = (y2 - y1) * 0.4
        dx = max(-limit, min(limit, vx * elapsed))
        dy = max(-limit, min(limit, vy * elapsed))
        return x1 + dx, y1 + dy, x2 + dx, y2 + dy

    @staticmethod
    def _nearby(previous, current):
        px1, py1, px2, py2 = previous
        cx1, cy1, cx2, cy2 = current
        old_height = py2 - py1
        new_height = cy2 - cy1
        if old_height <= 0 or not 0.7 <= new_height / old_height <= 1.4:
            return False
        dx = (cx1 + cx2 - px1 - px2) / 2
        dy = (cy1 + cy2 - py1 - py2) / 2
        return math.hypot(dx, dy) <= min(old_height, new_height) * 0.4

    def contains(self, track_id, now):
        record = self.tracks.get(track_id)
        return record is not None and now - record["last_seen"] <= self.max_gap_seconds


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
        self.person_confidence = float(rospy.get_param("~person_confidence", 0.35))
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
        self._person_tracks = PersonDetectionTracks(
            max(track_max_gap, float(rospy.get_param("~person_track_max_gap_seconds", 3.0))),
            max(0.1, track_iou),
        )
        self._follow_lock = threading.RLock()
        self._follow_enabled = False
        self._follow_track_id = ""
        self._follow_label = ""
        self._active_person_ids = set()
        self._person_crop_sent_at = {}
        self.confirmation_seconds = confirmation_seconds
        self._alert_pub = rospy.Publisher(self.alert_topic, String, queue_size=10)
        self._candidate_pub = rospy.Publisher(
            "/vision/person_candidates", String, queue_size=1
        )
        self._follow_state_pub = rospy.Publisher(
            "/vision/follow_state", String, queue_size=1, latch=True
        )
        self._follow_command_sub = rospy.Subscriber(
            "/vision/follow_command", String, self._on_follow_command, queue_size=10
        )
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
        self._publish_follow_state("OFF")

    def _publish_follow_state(self, status):
        with self._follow_lock:
            event = {
                "enabled": self._follow_enabled,
                "trackId": self._follow_track_id,
                "label": self._follow_label,
                "status": status,
                "timestamp": int(time.time() * 1000),
            }
        self._follow_state_pub.publish(String(data=json.dumps(event)))

    def _on_follow_command(self, message):
        try:
            command = json.loads(message.data)
            action = str(command.get("action", "")).upper()
            track_id = str(command.get("trackId", ""))
            with self._follow_lock:
                if action == "STOP":
                    self._follow_enabled = False
                    self._follow_track_id = ""
                    self._follow_label = ""
                    status = "OFF"
                elif action == "ENABLE":
                    self._follow_enabled = True
                    self._follow_track_id = ""
                    self._follow_label = ""
                    status = "WAITING_FOR_PERSON"
                elif action == "START":
                    if not track_id or not self._person_tracks.contains(track_id, time.monotonic()):
                        status = "TARGET_LOST"
                    else:
                        record = self._person_tracks.tracks[track_id]
                        self._follow_enabled = True
                        self._follow_track_id = track_id
                        self._follow_label = record["detection"][0]
                        status = "TARGET_SELECTED"
                else:
                    rospy.logwarn("Ignoring unknown Follow Me action: %s", action)
                    return
            self._publish_follow_state(status)
            rospy.loginfo("Follow Me mode=%s target=%s", status, track_id or "none")
        except (ValueError, TypeError, AttributeError) as error:
            rospy.logwarn("Invalid Follow Me command: %s", error)

    def _publish_person_candidates(self, image, people):
        candidates = []
        height, width = image.shape[:2]
        now = time.monotonic()
        for track_id, sent_at in list(self._person_crop_sent_at.items()):
            if now - sent_at > 300.0 and not self._person_tracks.contains(track_id, now):
                del self._person_crop_sent_at[track_id]

        with self._follow_lock:
            selected = self._follow_track_id
        visible_people = sorted(
            people,
            key=lambda item: (item["track_id"] == selected, item["detection"][1]),
            reverse=True,
        )[:3]
        for person in visible_people:
            track_id = person["track_id"]
            label, confidence, coordinates = person["detection"]
            x1, y1, x2, y2 = coordinates
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            candidate = {
                "trackId": track_id,
                "label": label,
                "confidence": round(confidence, 4),
                "bbox": {
                    "x": x1,
                    "y": y1,
                    "width": x2 - x1,
                    "height": y2 - y1,
                },
            }
            if track_id not in self._person_crop_sent_at:
                crop = image[y1:y2, x1:x2]
                crop_h, crop_w = crop.shape[:2]
                scale = min(200.0 / crop_w, 240.0 / crop_h, 1.0)
                if scale < 1.0:
                    crop = cv2.resize(
                        crop,
                        (max(1, int(crop_w * scale)), max(1, int(crop_h * scale))),
                        interpolation=cv2.INTER_AREA,
                    )
                ok, encoded = cv2.imencode(
                    ".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 45]
                )
                if ok:
                    candidate["imageBase64"] = base64.b64encode(
                        encoded.tobytes()
                    ).decode("ascii")
                    self._person_crop_sent_at[track_id] = now
            candidates.append(candidate)
        self._candidate_pub.publish(
            String(
                data=json.dumps(
                    {
                        "robotCode": self.robot_code,
                        "timestamp": int(time.time() * 1000),
                        "imageWidth": width,
                        "imageHeight": height,
                        "candidates": candidates,
                    },
                    separators=(",", ":"),
                )
            )
        )
        with self._follow_lock:
            self._active_person_ids = {person["track_id"] for person in visible_people}
            selected = self._follow_track_id
            enabled = self._follow_enabled
            if enabled and selected and not self._person_tracks.contains(selected, time.monotonic()):
                self._follow_track_id = ""
                self._follow_label = ""
                status = "TARGET_LOST"
            else:
                status = None
        if status:
            self._publish_follow_state(status)
        return candidates

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
        self, image, fire, fall, people, fire_track, fall_track
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
            elif people:
                fall_status = "PERSON: %d candidate(s)" % len(people)
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
            overlays = [
                (fire, (0, 0, 255), fire_track["track_id"], ""),
                (fall, (255, 80, 0), fall_track["track_id"], ""),
            ]
            with self._follow_lock:
                selected_track_id = self._follow_track_id
            for person in people:
                detection = person["detection"]
                is_selected = person["track_id"] == selected_track_id
                overlays.append(
                    (
                        detection,
                        (255, 255, 0) if is_selected else (0, 200, 0),
                        person["track_id"],
                        "FOLLOW " if is_selected else "",
                    )
                )
            for detection, color, track_id, prefix in overlays:
                if detection is None:
                    continue
                label, score, coordinates = detection
                x1, y1, x2, y2 = coordinates
                cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)
                caption = "%s%s %.0f%%" % (prefix, label, score * 100.0)
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

    def _encode_alert_crop(self, image, detection):
        height, width = image.shape[:2]
        x1, y1, x2, y2 = detection[2]
        box_width = max(1, x2 - x1)
        box_height = max(1, y2 - y1)
        margin_x = int(box_width * 0.15)
        margin_y = int(box_height * 0.15)
        left = max(0, int(x1) - margin_x)
        top = max(0, int(y1) - margin_y)
        right = min(width, int(x2) + margin_x)
        bottom = min(height, int(y2) + margin_y)
        crop = image[top:bottom, left:right]
        if crop.size == 0:
            raise ValueError("detection box produced an empty alert crop")

        crop_height, crop_width = crop.shape[:2]
        scale = min(1.0, 640.0 / max(crop_width, crop_height))
        if scale < 1.0:
            crop = cv2.resize(
                crop,
                (max(1, int(crop_width * scale)), max(1, int(crop_height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        for quality in (72, 58, 44):
            encoded, buffer = cv2.imencode(
                ".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            if encoded and len(buffer) <= 300 * 1024:
                return base64.b64encode(buffer).decode("ascii")
        raise ValueError("compressed alert crop exceeds 300 KB")

    def _emit(self, event_type, detection, track_id, image):
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
            "imageContentType": "image/jpeg",
        }
        try:
            payload["imageBase64"] = self._encode_alert_crop(image, detection)
        except (ValueError, cv2.error) as error:
            rospy.logerr("Could not attach safety alert image: %s", error)
        self._alert_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        image_size = len(payload.get("imageBase64", ""))
        rospy.logwarn(
            "Safety alert published: type=%s track=%s image_base64_chars=%s",
            event_type,
            track_id,
            image_size,
        )

    def _inference_loop(self):
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="vision-fire") as fire_worker:
            self._process_frames(fire_worker)

    def _detect_fire(self, image):
        return self._best_detection(
            self._detections(self._fire_model, image, {"fire", "smoke", "flame"}, self.fire_confidence)
        )

    def _process_frames(self, fire_worker):
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
            fire_result = fire_worker.submit(self._detect_fire, image)
            fire_inference_ok = True
            fall_inference_ok = True
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
                    min(self.fall_confidence, self.person_confidence),
                )
                fall_labels = {
                    "fallen", "fall", "person fall", "human fall", "falling person"
                }
                standing_labels = {"standing", "person standing", "standing person"}
                sitting_labels = {"sitting", "person sitting", "sitting person"}
                fall = self._best_detection(
                    [d for d in posture_detections if _normalise_label(d[0]) in fall_labels and d[1] >= self.fall_confidence]
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
            person_detections = [
                detection
                for detection in posture_detections
                if _normalise_label(detection[0]) in standing_labels | sitting_labels
                and detection[1] >= self.person_confidence
            ] if fall_inference_ok else []
            people = self._person_tracks.update(person_detections, processed_at)
            self._publish_person_candidates(image, people)
            try:
                fire = fire_result.result()
            except Exception as error:
                fire_inference_ok = False
                rospy.logerr_throttle(30, "Fire/smoke inference failed: %s", error)
            fire_track = self._fire_track.update(
                fire if fire_inference_ok else None, time.monotonic()
            )
            fall_track = self._fall_track.update(
                fall if fall_inference_ok else None, processed_at
            )
            self._update_preview(
                image,
                fire,
                fall,
                people,
                fire_track,
                fall_track,
            )

            if fire_track["newly_confirmed"] and fire is not None:
                self._emit("FIRE", fire, fire_track["track_id"], image)
            if fall_track["newly_confirmed"] and fall is not None:
                self._emit("PERSON_FALL", fall, fall_track["track_id"], image)
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
