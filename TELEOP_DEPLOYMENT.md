# Teleoperation deployment — 2026-11-14

Deployed backend on 34.172.236.168:8081 (`robot-y-ta.service`), Pi bridge in
`ros-noetic` on 172.18.1.105, and admin APK on Pixel 7 `2A211FDH200L9J`.
Lenovo robot tablet was not reinstalled.

Confirmed production STOMP probe: `ready: true`, `robotState: MAPPING`.
Three zero-velocity commands were accepted through the authenticated operator
WebSocket; an independent ROS subscriber received a zero Twist on
`/cmd_vel/joystick`. No nonzero velocity was sent by this deployment check.
This verifies transport and routing, not physical driving or wheel directions.

`/cmd_vel` now has the existing cmd_vel_mux as its sole publisher. Joystick
bridge publishes `/cmd_vel/joystick`; the mux subscribes there. Priority and
source timeouts are unchanged. Docking output config is now
`/cmd_vel/docking`; future move_base starts from localization.launch remap
cmd_vel to `/cmd_vel/navi`. The mux launch loads config under the node's private
namespace, matching the mux implementation.

The duplicate localize_manager launch created earlier (PID 3736) was stopped.
The team's manager launch and its current gmapping session were retained.
Only idle moving_server was restarted, as authorized, to load docking routing
and start the mux. ROS core and hardware/sensor processes were not restarted.

Persistent launch wiring: moving_server.launch includes cmd_vel_mux.launch;
localize_manager.launch includes teleop_bridge.launch. The current teleop
bridge was started independently using rosrun, avoiding a restart of the active
manager/mapping session. On a future manager restart, the bridge singleton lock
rejects any extra instance while that standalone process remains alive. For a
planned clean full launch, stop that standalone bridge first; do not start a
second manager or core.

Runtime compatibility fixes: websocket-client 0.53 callback binding on Ubuntu
requires function wrappers; manager state is latched on transitions, so the
bridge requests fresh latched samples to validate liveness without changing the
manager. Mobile shows actual Pi state and exposes Save/Stop while MAPPING,
even if no mapping command was previously stored on the server.

Fixed backend mapping-command creation: a newly constructed PENDING command
previously hit an idempotency return before repository.save. New requests now
save before returning. Three focused unit checks passed (new persistence,
idempotent retry, replacement of completed command). No live Start/Save/Stop
command was used to validate this fix against the active mapping session.

Validation: backend package successful; Flutter analyze clean; Android APK
build/install successful; eight offline watchdog checks passed. The live zero
probe is in tests/live_zero_probe.py and uses no third-party WebSocket client.

Original backend rollback JAR:
`/opt/robot-y-ta/robot-api.jar.before-teleop-20261114T155343Z`.
Final JAR SHA256:
`c2ddafadd14f91078240aa233db821d9b86739ebcf3c982dd129d0834b99647e`.
Pi file backups have `.before-teleop-20261114T155401Z` or
`.before-teleop-20261114T155643Z` suffixes. Modified Pi source/config files were
pulled back to `pi_overlay/` beside this report; no robot credentials included.
