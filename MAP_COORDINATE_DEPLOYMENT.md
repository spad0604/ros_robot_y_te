# Map identity and delivery coordinate deployment — 2026-11-17

## Change

- The admin app sends the selected `mapId` with `mapName` and only enables
  coordinate selection after Pi confirms that same `mapId` as `APPLIED`.
- The backend resolves selections by exact map ID, rejects stale map versions,
  and rejects coordinate deliveries unless that exact map version is active.
- The Pi order listener requires a map ID in selection state and preserves it
  when applying localization. The delivery executor already checks the task
  map ID against the active selection before publishing a goal.

## Deployment

- Backend service: `robot-y-ta.service` on `34.172.236.168:8081`.
- JAR SHA-256: `3efcdbebe58e664b23e9eff3ca8fc05adae574549e2cc9a99689b7dde300fa11`.
- Previous JAR backup: `/opt/robot-y-ta/robot-api.jar.before-map-id-identity-20261117T134848Z`.
- Pi package: `/root/ros_ws/cleaning_robot/pi_ws/src/robot_system/localize_manager`.
- Pi listener backup: `/root/ros_ws/backups/map-id-identity-20261117T134848Z/localize_manager/order_listener.py.before`.
- Pi source SHA-256: `937e9423b5c18cd387d0afc0a468fccde190b9cdeda435370f38ba1d2368615b`.
- Admin APK SHA-256: `67ae6f1372992617237c587081e032dadbdacd4eb92e03007126e4b8632d44c7`;
  installed on Pixel 7 `2A211FDH200L9J`.

## Verification

- Backend service active; HTTPS `/api/v1/health` returned `UP`.
- `catkin_make --pkg localize_manager` completed on the Pi.
- The `order_listener` was respawned by its existing roslaunch parent and
  authenticated. `/robot/orders` is connected to `/delivery_executor`.
- No delivery was created and no movement goal was sent during deployment.
- Physical navigation remains dependent on the Pi having localization,
  odometry, camera and `move_base` available.
