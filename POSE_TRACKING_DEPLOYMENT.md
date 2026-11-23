# Pose tracking backend deployment — 2026-11-17

Backend pose endpoints and WebSocket authorization were deployed to the VM
service `robot-y-ta.service` at `34.172.236.168:8081`.

- New JAR SHA-256: `d1176a4e689052355fd32b6e7c254720211f36c0b0b86146731d9e043b7541ca`
- Previous JAR backup: `/opt/robot-y-ta/robot-api.jar.before-pose-tracking-20261117T020945Z`
- Service status after restart: `active`
- Local and external `/api/v1/health`: HTTP `200`, `UP`
- Unauthenticated reads of `/pose/latest` and `/pose/history`: HTTP `401`
- Existing service on port `8080` remained listening

The Pi uploader and admin APK have not been deployed in this step. Pose data
will begin arriving after the Pi overlay is installed and localization is
active. The latest pose and bounded journey trail live in backend memory and
are cleared by a backend restart.
