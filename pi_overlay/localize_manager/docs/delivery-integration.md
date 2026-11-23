# Delivery execution through moving_server

Admin selects the robot's current saved map and waits for the Pi map-selection
acknowledgement. Admin chooses **only the delivery point**, then creates a task.
The task carries robotCode, mapId, dropoffX/Y (metres in map frame), optional
heading dropoffYaw (radians, default 0), recipient and note.

The existing order_listener still publishes JSON on /robot/orders. The new
standalone delivery_executor uses that topic as a wake-up signal and also polls
the authenticated robot queue to recover missed/latched events. It does not rely
on the listener's remembered seen IDs for execution or acknowledgement.

## Motion contract

The adapter sends a standard MoveCommandActionGoal to
/moving_server/move_command/goal with mode MODE_MOVE_TO_POINT, a map-frame
PoseStamped and a 300 second timeout. It consumes the matching /feedback,
/result and /status topics and cancels through /cancel with a **specific** GoalID.
It never publishes Twist, talks to motor serial ports or bypasses moving_server.

The standard action wire protocol is used so an execution UUID can be persisted
as a deterministic GoalID **before** publication. On restart the adapter cancels
that ID and reconciles the execution instead of resending the destination.
A disk journal in ~/.ros/delivery_executor.json uses atomic replacement/fsync and
mode 600; a local singleton lock prevents duplicate executors.

Readiness requires LOCALIZING, a matching APPLIED map selection, a map grid, a
fresh map-to-base_link transform, live moving_server action links and status,
idle gateway, and a live idle move_base. Targets outside the map, unknown cells
or occupied cells fail before any action goal is published. It does not start
localization, initialize robot pose, switch maps or claim AMCL/GMCL convergence.
If needed the operator initializes the pose in RViz before creating a delivery.
Existing TEB/planner settings and control algorithms remain untouched.

## Backend ownership and results

Robot-authenticated endpoints:

```
GET  /api/v1/robots/{code}/deliveries/pending
GET  /api/v1/robots/{code}/deliveries/active
POST /api/v1/robots/{code}/deliveries/{taskId}/claim      {executionId}
POST /api/v1/robots/{code}/deliveries/{taskId}/heartbeat  {executionId, sequence, ...feedback}
POST /api/v1/robots/{code}/deliveries/{taskId}/result     {executionId, sequence, outcome, resultCode, ...}
```

A transactional robot row lock serializes claims and map changes. There is one
RUNNING delivery per robot. A claim must match the robot, selected map, dispatch
flag and due time; repeating the same execution is idempotent. Map/start/stop
mapping requests reject while a delivery remains RUNNING or awaiting stop.
Feedback/result sequence numbers reject older updates. New coordinate delivery
requests enable dispatch; pre-deployment rows and legacy waypoint-only tasks
remain inactive until the operator recreates them explicitly. Request keys are
persisted for idempotent HTTP retries.

Transitions: CREATED → RUNNING (DISPATCHING then gateway feedback) → COMPLETED,
FAILED or CANCELLED. Completion requires action status SUCCEEDED, success=true
and RESULT_SUCCESS=0. COMPLETED means the robot arrived at the destination;
physical handover is still confirmed by the staff.

Operator cancel on a queued task is immediate. Cancel on a RUNNING task sets
cancellationRequested and CANCELLING; the task remains RUNNING until the Pi
receives terminal action status/result. This cancels the moving_server goal,
which also cancels its move_base goal according to the gateway contract.
Sending a zero Twist would not be a substitute for this cancellation.

A separate watchdog cancels the owned action if the backend is unreachable for
3 seconds, localization/map/TF is lost, or execution times out. The server lease
is 10 seconds but does not automatically reassign/replay an expired execution.
Results persist locally and retry after reconnection. Interrupted executions
are stopped and reported FAILED/CANCELLED, never automatically resumed.

## App and installation

Task list/detail show navigation feedback, distance, progress and cancellation
acknowledgement. Task events update the app; a 2 second REST poll is the fallback.
Cancel is available for running tasks and asks for confirmation.

Install delivery_executor.py and delivery_engine.py in localize_manager/scripts,
register both in catkin_install_python, install delivery_executor.launch in
launch/, and include it exactly once from the existing manager launch. Runtime
uses geometry_msgs/actionlib_msgs/nav_msgs/tf2_ros/moving_server_msgs and the
existing robot credential file. No ROS core or movement algorithm change is
required. In an already-running mapping session the executor can be started on
its own; the package launch include applies to subsequent full launches.

## Validation

Backend tests cover claim ownership, map/legacy rejection, single active task,
idempotency, feedback ordering, cancellation and result validation. Offline
Python tests cover no-replay/recovery, API loss, specific-goal cancellation,
wire message fields and map-coordinate validation. The ROS integration test in
tests/ros_delivery_integration.py uses /delivery_wire_test/* with a mock action
server; it checks real action transport for completion and cancellation and
never publishes motor velocities or sends a goal to the physical gateway.
