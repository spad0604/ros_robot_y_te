# Follow Me motion path

The admin tablet selects a visible person by `trackId`. The authenticated
server relays that selection to the Pi, where `vision_alert_detector.py`
confirms the track. `follow_person_pose.py` matches the selected bbox bearing
to `/scan` and publishes a fresh `PoseStamped` on `/person_tracker/pose` in
`base_footprint`. `follow_me_motion.py` sends
`MODE_TRACK_PERSON` to `/moving_server/move_command`; the existing
`moving_server` and `cmd_vel_mux` remain the only motion path.

```text
tablet selection -> server/STOMP -> /vision/follow_command
    -> detector-selected track + fresh camera bbox + /scan + TF
    -> /person_tracker/pose -> moving_server action -> cmd_vel_mux
```

## Runtime behavior

- The robot only starts moving after an explicit tablet selection.
- Enabling Follow Me starts localization with the map currently selected in
  the admin app. `order_listener.py` exposes the successfully applied map as
  `/localize_manager/active_map_name`; Follow Me calls
  `/localize_manager/start_localization` when localization is not active.
- A person selection received while localization is starting stays queued. The
  robot sends `MODE_TRACK_PERSON` to `moving_server` only after the state topic
  reports `LOCALIZING`, the selected target and LiDAR pose are fresh, and the
  motion gateway is idle.
- If no map is selected, map saving is in progress, or localization fails, the
  tablet receives a matching status and no movement goal is sent. Turning
  Follow Me off cancels its movement goal but leaves localization running.
- Starting localization while the robot is mapping switches the manager to
  localization on the selected saved map; any unsaved mapping session is
  therefore ended. Save the map before enabling Follow Me if those scans need
  to be kept.
- A pose is emitted only while the selected track and LaserScan are fresh,
  the scan has a TF transform, and at least two LiDAR returns support the
  target bearing.
- Missing range data, a lost track, or stale pose cancels the Follow Me
  action. No fallback or direct `cmd_vel` publication is used.
- Each Follow Me action has a unique ID journaled on the Pi. If the adapter
  restarts, it cancels only its previous action ID before accepting another
  follow request.
- The action is rejected while `moving_server` is unavailable or busy. The
  delivery executor also waits while Follow Me is starting or active, so a
  newly dispatched order cannot intentionally preempt the selected follow.
- The controller uses the configured 1.0 m standoff and follows until stopped,
  the track is lost, or its one-hour action timeout is reached.

## Pi launch parameters

`localize_manager.launch` includes the bridge, target pose estimator, and
motion adapter through `follow_me_bridge.launch`:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `scan_topic` | `/scan` | Planar LiDAR scan |
| `base_frame` | `base_footprint` | Target pose and moving controller frame |
| `pose_topic` | `/person_tracker/pose` | Pose input used by moving_server |
| `camera_horizontal_fov_degrees` | `74.0` | Horizontal camera field of view |
| `camera_yaw_offset_degrees` | `0.0` | Camera heading relative to `base_footprint` |
| `start_localization_service` | `/localize_manager/start_localization` | Starts localization on enable |
| `active_map_name_param` | `/localize_manager/active_map_name` | Selected, applied map published by `order_listener.py` |
| `localization_timeout_seconds` | `45.0` | Maximum wait for localization state to become `LOCALIZING` |

The camera field of view and yaw offset must match the physical camera mount.
The 74 degree default is a bring-up estimate; calibrate it on the Pi before
robot movement. The estimator intentionally stops when camera bearing cannot
be associated with valid scan returns.

Tablet state is relayed on `/topic/robots/{robotCode}/follow/state`, including
`LOCALIZATION_STARTING`, `MAP_NOT_SELECTED`, `MAP_SAVE_IN_PROGRESS`,
`LOCALIZATION_FAILED`, `STARTING`, `FOLLOWING`, `MOTION_BUSY`,
`MOTION_UNAVAILABLE`, `TARGET_RANGE_UNAVAILABLE`, and stop/failure states.
