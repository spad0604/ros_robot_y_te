# Robot vision safety alerts

`localize_manager.launch` starts `vision_alert_detector.py` and
`robot_alert_uploader.py`. By default the detector captures the Pi's `/dev/video0`
camera, keeps only the newest frame in a one-slot ring, and runs both Ultralytics
models against that same frame in one worker. It is rate-limited to 2 inference
frames per second; camera frames are never queued behind slow inference. The Pi
camera was identified as `XWF-1080P`, and a live frame could be read from it.

The default model files are installed in
`localize_manager/models/fire_smoke.pt` and
`localize_manager/models/huma_fall.pt`. They are copied from the model files at
the repository root. Model confidence defaults to 0.55. The fall model's
`standing` class is drawn in the VNC preview as well as `fallen`; standing does
not trigger an alert. Fire/smoke and fallen detections must keep the same
IoU-matched track ID for 3 seconds before they alert. A track may miss
detections for up to 1.25 seconds without losing its ID. Each track publishes
at most one alert.

## Camera and runtime requirements

The detector captures the camera directly through OpenCV/V4L2, so it does not
require a separate ROS camera publisher. Override the device in a full launch
with:

```bash
roslaunch localize_manager localize_manager.launch camera_device:=/dev/video1
```

If the camera is published by another ROS node instead, set
`camera_device:=` (empty) and set `image_topic:=/your/camera/image_raw`.

## Live VNC preview

The detector can show its live frame with the current best fire/smoke, fallen,
standing, and sitting boxes overlaid. Safety boxes show their track ID and
confirmation timer. Toggle it at runtime from the Pi terminal; no full launch
restart is needed:

```bash
docker exec ros-noetic bash -lc 'source /opt/ros/noetic/setup.bash && source /root/ros_ws/cleaning_robot/pi_ws/devel/setup.bash && ROS_MASTER_URI=http://172.18.1.105:11311 rosparam set /vision_alert_detector/show_window true'
```

The `MediBot safety detection` window appears in VNC. Turn it off with the same
command and `false` instead of `true`. The preview is disabled by default, so a
future full launch does not depend on a VNC display.

On the Pi, the detector uses the isolated runtime at
`/root/ros_ws/vision_runtime/site-packages` through
`scripts/vision_python.sh`; it does not change ROS/system Python packages.
The detector is respawned if inference exits, and it is not marked `required`,
so an inference failure does not terminate the rest of the robot bring-up.

The Pi runtime uses Python 3.8 ARM64 wheels: PyTorch 2.4.1, torchvision
0.19.1, Ultralytics 8.4.51, NumPy 1.24.4, and the model inference
dependencies. Polars is an optional Ultralytics utility on Python 3.8 and is
not needed by this inference-only node.

## Alert path

The detector publishes JSON on `/robot/alerts`; the uploader posts each event
to `POST /api/v1/robots/{robotCode}/alerts`:

```json
{
  "severity": "CRITICAL",
  "type": "PERSON_FALL",
  "message": "Robot phát hiện người có dấu hiệu bị ngã (độ tin cậy 91%).",
  "sourceEventId": "MEDIBOT-01:PERSON_FALL:<unique-id>",
  "imageContentType": "image/jpeg",
  "imageBase64": "<JPEG crop of the confirmed detection box>"
}
```

Fire or smoke uses type `FIRE`; a fall uses `PERSON_FALL`. The API deduplicates
by `sourceEventId`, stores the JPEG crop under the server data directory,
publishes its public `imageUrl` on `/topic/alerts`, and sends an FCM notification
with that image to registered staff devices and the matching robot tablet. The
crop is limited to 300 KB; no video or continuous frame stream is uploaded. The
FCM image URL is `GET /api/v1/alerts/images/{uuid}`. Staff sign-in and robot
auto-login register each device's FCM token at
`POST /api/v1/notifications/device-token`; token registration requires the
device's normal authenticated session. Android notifications show the image in
the background, and each app displays a local image notification while in the
foreground. The tablet's existing safety alarm continues until the alert is
acknowledged. The uploader retries network and server errors with the same event
ID and image.

The detector only publishes event starts. It never uploads camera frames or
video, and losing a local detection track does not acknowledge the saved
server alert.
