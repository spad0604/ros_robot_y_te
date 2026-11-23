# Admin map and delivery integration report

**Date:** 2026-11-14  
**Pi source checkout:** `cleaning_robot`, HEAD `0a2af83`, with local modifications

## Admin workflow

1. In **Nhiệm vụ → Tạo nhiệm vụ giao đồ**, choose the robot and its current map.
2. The app requests localization and waits until the Pi reports `APPLIED` before enabling destination selection.
3. Choose only the delivery coordinate. The request contains `mapId`, `robotCode`, `dropoffX`, and `dropoffY`; pickup coordinates are not required.
4. Creating the task stores it as `CREATED`. The Pi listener polls the API and publishes the matching task as JSON on `/robot/orders`.

Map scanning is available from **Nhiệm vụ → Quét / quản lý map robot**. The admin can start scanning, save a named map, and stop scanning. The stop bridge checks `/localize_manager/state` and calls `/localize_manager/stop` only when the state is `MAPPING`.

## Pi map setup

- `localize_manager/config/localize_manager.yaml` sets `map_directory: maps`.
- Maps live at `pi_ws/src/robot_system/localize_manager/maps/<map-name>/<map-name>.yaml` with its referenced `.pgm`, `.png`, or `.bmp` image.
- Pi currently has `test_01` and `test_02`; localization was running from `test_02` during deployment.
- `localize_manager.launch` includes `server_uploaders.launch` and `order_listener.launch`.
- The map uploader scans every 300 seconds. A newly saved map can take up to five minutes to appear in the admin map list.
- The listener uses the robot account config at `/root/.config/medibot/order_listener.json` (mode `600`); credentials are not included in this report or source snapshot.

Existing ROS interfaces used by the bridge:

```text
/localize_manager/start_localization   StartLocalization(map_name)
/localize_manager/start_mapping        StartMapping()
/localize_manager/save_map             SaveMap(map_name)
/localize_manager/stop                 Stop()
```

## Deployment and preservation

- The API JAR was deployed to the VM and `robot-y-ta.service` is active; `/api/v1/health` returned HTTP 200.
- `order_listener.py` and `order_listener.launch` were deployed to the Pi package. The listener was respawned by its existing roslaunch and authenticated to the API.
- No ROS core/container restart or localization/motion service call was made. The active map remained `test_02`.
- Admin debug APK was installed on the connected Pixel 7. The Lenovo robot tablet was left unchanged.
- A source snapshot of the Pi checkout is stored separately on the admin workspace; see the delivery note for its path and checksum.

## Known boundary

`/robot/orders` currently has `/order_listener` as its publisher and no subscriber. The bridge delivers orders to ROS but does not call navigation, publish velocity, or change task state. The movement team must connect an existing control consumer to the order payload before orders can drive the robot.

No live start-mapping, save-map, map-switch, or delivery command was issued during deployment, so the running robot state was not changed.
