# ROS Pi map selection and order intake

`order_listener.py` polls the API for map selection changes and delivery tasks.
When an admin selects a map, the API creates a pending selection for that robot.
The Pi listener calls `/localize_manager/start_localization` with the selected
`mapName`, then reports `APPLIED` or `FAILED` to the API. The admin form waits
for `APPLIED` before allowing coordinate selection.

The listener also polls for `DELIVERY` tasks with status `CREATED` every two
seconds and publishes each new, robot-matched task as JSON on `/robot/orders`
(`std_msgs/String`). Coordinate tasks are published only when their robot and
map version match the map currently acknowledged by localization.

The admin can also issue `START_MAPPING`, `SAVE_MAP`, and `STOP_MAPPING`
commands. The Pi dispatches them to `/localize_manager/start_mapping`,
`/localize_manager/save_map`, and `/localize_manager/stop`. Stop is guarded by
the existing `/localize_manager/state` topic and is sent only while the manager
reports `MAPPING`. Saved maps are uploaded by the existing map uploader.

The listener does not call `moving_server`, publish to `/cmd_vel`, or change
task status. Navigation remains separate.

On the Pi, put the robot API account in
`~/.config/medibot/order_listener.json` with mode `600`:

```json
{"username":"MEDIBOT-01","password":"..."}
```

For a current ROS session, start it independently with:

```bash
roslaunch localize_manager order_listener.launch
```

`localize_manager.launch` includes this listener, so it starts with the normal
localization/uploader launch. Run either the normal launch or the standalone
listener launch, not both at once.

The admin picker reads the selected map YAML (`resolution` and `origin`) and
sends the delivery coordinate in the map's world frame. This listener passes
the API task through unchanged after the corresponding map is active.

## Mobile manual driving

See [TELEOP.md](TELEOP.md) for the mapping-only WebSocket bridge, mux gates,
installation artifacts and offline watchdog checks. It is a separate node; do not
launch a second localize_manager or ROS core to enable it.

## Delivery execution

See [DELIVERY.md](DELIVERY.md) for the delivery executor, robot claim/feedback/result
API and action cancellation. The existing order topic remains available; execution
is now handled by the separate delivery_executor node through moving_server.

## Fire, smoke and fall detection

See [VISION_ALERTS.md](VISION_ALERTS.md). A single 2 FPS inference worker gives
the same newest camera frame to `fire_smoke.pt` and `huma_fall.pt`, then
publishes `FIRE` or `PERSON_FALL` on `/robot/alerts` for the existing server
uploader. Set `image_topic` to the ROS image topic actually published by the
robot's camera driver.
