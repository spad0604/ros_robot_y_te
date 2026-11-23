# Delivery deployment — 2026-11-15 (Asia/Bangkok)

Backend deployed on 34.172.236.168:8081, service robot-y-ta.service health UP.
JAR SHA256: 2350733c78864bfa99dd45c32a2676c8f5040c00472d7b1cdc74d1120ee0a608.
DB backup: /opt/robot-y-ta/backups/robot-db-before-delivery-20261114T170325Z.dump.
Pre-deployment JAR retained as /opt/robot-y-ta/robot-api.jar.before-delivery-20261114T170325Z.

Admin APK installed through ADB on Pixel 7 2A211FDH200L9J. Robot tablet unchanged.
Task list/detail update via task events with REST fallback; running tasks can be
cancelled, with status remaining RUNNING/CANCELLING until the robot confirms stop.

Pi package: /root/ros_ws/cleaning_robot/pi_ws/src/robot_system/localize_manager
inside ros-noetic on 172.18.1.105. Installed delivery_engine.py,
delivery_executor.py and delivery_executor.launch, CMake registration and runtime
package dependencies. Existing localize_manager.launch now includes the executor
for subsequent full launches. During this deployment the new executor was started
alone via rosrun; existing manager, moving_server, move_base, sensors and core
were not restarted. Singleton lock prevents an extra executor from registering.
For a clean future launch, stop that standalone executor before relaunching the
manager; do not start a second manager/core.

Live checks:

* Robot API: active deliveries 0, dispatch-enabled pending deliveries empty.
* Selected map giap_test, APPLIED, mapId 35fc19e2-2009-4b33-90c1-b258d1836896.
* Actual manager state LOCALIZING; map frame map; gateway idle.
* Exact executor admission check returned (True, '').
* /delivery_executor publishes only action goal/cancel and rosout; gateway
  subscribes on /moving_server/move_command/goal and /cancel.
* Existing planner: teb_local_planner/TebLocalPlannerROS, max_vel_y=0.3.

Validation passed: all 15 backend tests, 22 offline Python checks, Flutter
analysis/build/install, catkin package build. Isolated real ROS action transport
checks returned ROS_ACTION_E2E COMPLETED PASS and CANCELLED PASS under
/delivery_wire_test/*. No physical delivery goal or fabricated destination was
sent during deployment. Staff should create a new order at a chosen delivery
point to validate actual navigation/arrival on the robot.

Pre-deployment orders are not automatically enabled for motion. New coordinate
DELIVERY tasks carry dispatchEnabled=true. Backups of Pi CMake/package/launch
files have suffix .before-delivery-20261114T170121Z. Updated Pi source/config
snapshot was pulled into pi_overlay/ beside this report, without credentials.
See DELIVERY.md for the complete API, journal and cancellation contract.
