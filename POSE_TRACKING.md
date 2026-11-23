# Robot pose and journey tracking

The admin app's **More → Theo dõi robot** view displays the robot's live pose,
recent travelled trail and the active delivery destination on the saved map.
The pose is sent by the Pi only while `localize_manager` reports `LOCALIZING`.

## Data flow

1. `robot_pose_uploader.py` reads `map → base_link` from TF at 2 Hz. It reads
   the robot's currently applied map selection using the robot credentials in
   `~/.config/medibot/order_listener.json`.
2. The Pi posts `{frameId, mapId, mapName, x, y, yaw}` to the authenticated
   `POST /api/v1/robots/{code}/pose` endpoint.
3. The backend publishes each pose to
   `/topic/robots/{code}/pose`. It retains the latest pose and a bounded recent
   path in memory, exposed through `GET .../pose/latest` and
   `GET .../pose/history`.
4. Mobile subscribes to that topic and also polls the latest pose every two
   seconds as a reconnect fallback. It overlays the current pose, travelled
   trail, active delivery target and delivery progress on the selected map.

The pose endpoints require the robot's own JWT for uploads, and an ADMIN or
COORDINATOR JWT for reading. The server keeps up to 3,000 movement-filtered
path samples per robot in memory; this is a recent journey trail, not durable
history across a backend restart. The existing ROS motion/control topics and
controllers are not changed.

## Local validation and deployment

The node is included in `server_uploaders.launch`, which is already included
by `localize_manager.launch`. Build the catkin workspace on Pi after copying
the overlay changes, then start the same full launch as before. No extra ROS
core or motion service should be started for pose tracking.
