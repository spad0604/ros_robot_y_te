# Mobile manual driving during mapping

Implementation: admin `RobotMappingPage` → authenticated STOMP WebSocket `/ws`
→ `RobotTeleopController` → robot's `/user/queue/teleop`
→ standalone `mobile_teleop_bridge` → `/cmd_vel/joystick` (`geometry_msgs/Twist`)
→ existing `cmd_vel_mux` → `/cmd_vel` → existing base driver.

The mecanum controls send forward/backward, sideways and yaw. Defaults are
0.15 m/s translation and 0.3 rad/s yaw; server and Pi enforce maximum
translation magnitude 0.20 m/s and yaw 0.40 rad/s. No navigation action,
motor serial code, mux priority or ROS core configuration is changed.

## Gates and stopping

* Pi requires a fresh `/localize_manager/state` equal to `MAPPING` (1.5 seconds).
  The installed manager emits latched state on transitions only, so a background
  check requests a fresh latched sample every cycle; timeout disables driving.
  It does not infer state from the last API mapping action.
* Pi verifies `/cmd_vel_mux` subscribes to `/cmd_vel/joystick` AND publishes
  `/cmd_vel`. Override private `mux_node` only if the existing mux has another name.
  A direct base subscriber to `/cmd_vel` does not satisfy this requirement.
* Docs set joystick priority 20, navigation 40, tracking 60, docking 80,
  emergency 100. These priorities are preserved. Any messages on the higher
  priority inputs block manual controls until they have been quiet for 0.6 s.
* Pi reports readiness every 0.5 s. Server readiness expires after 1.5 s;
  mobile readiness expires after 1 s. UI enables hold controls only with readiness.
* Server grants a 0.5 s lease to one STOMP operator session. Robot must log in
  as matching ROBOT code; operator requires ADMIN or COORDINATOR. Queues are
  per authenticated user; clients cannot publish directly into teleop queues.
* App sends drive frames at 10 Hz only while held. Pointer up/cancel/leaving
  the control, app background, route disposal or mapping API operation releases
  driving. Reconnect never resumes a held command.
* Commands carry increasing sequence numbers at mobile and server to reject reordered frames.
  They are ephemeral, stamped, never latched/stored/replayed; they expire
  within 250 ms. Pi uses a monotonic local deadline and emits zero on loss of
  permission, state, connection or deadline. Its velocity timer never performs
  network IO. Existing mux timeout is 0.3 s; existing base timeout is 0.5 s.
* Server, mobile and Pi wall clocks must be synchronized for freshness checks.
  An incorrect clock rejects driving instead of extending a command lifetime.
* Stop button stops manual driving only. It is not a replacement for the existing
  physical emergency stop or the moving_server emergency stop.

## Map workflow and documentation discrepancy

Start mapping through existing API and wait for actual Pi state MAPPING.
Drive manually and inspect `/map` in RViz. This change does not add a live map
viewer to mobile. Send SAVE_MAP with a name after visual inspection.
The supplied README says SAVE_MAP stops gmapping and returns IDLE, but the
inspected Pi implementation explicitly preserves gmapping and MAPPING after save.
The bridge follows actual state, so driving is disabled if the installed manager
returns IDLE after save; otherwise use the existing STOP_MAPPING command separately.

## Installation artifacts (not automatically launched)

Local code is prepared; this document does not certify live robot operation.
Deploy backend JAR and admin APK together, and install `teleop_bridge.py` into
`localize_manager/scripts/` (executable), and `teleop_bridge.launch` into `launch/`.
Add the Python script to the package's `catkin_install_python(PROGRAMS ...)` list,
then build with the team's normal catkin process. Runtime dependency:
`python3-websocket` / Python `websocket-client` (not the unrelated websocket package).
Use the existing mode-600 robot credential file with username/password.

The team can include `teleop_bridge.launch` exactly once in its existing launch,
using the same ROS_MASTER_URI/ROS_IP as the sensors, manager and mux. The bridge
also takes a file lock to reject a second local instance; it has no respawn loop.
Do not start another localize_manager launch or a new ROS master to add this bridge.

Before live use, check with the team that there is one manager/listener launch,
manager state and mux are healthy, drive axes match the physical base, and verify
release/disconnection stopping with the robot secured. No motion was exercised by
local checks; watchdog tests use fake ROS modules/publishers and no network.
