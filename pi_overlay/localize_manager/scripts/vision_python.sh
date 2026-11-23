#!/bin/sh

# Keep the AI dependencies isolated from ROS/system Python packages.
VISION_SITE_PACKAGES="${MEDIBOT_VISION_SITE_PACKAGES:-/root/ros_ws/vision_runtime/site-packages}"
export PYTHONPATH="${VISION_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
exec /usr/bin/python3 "$@"
