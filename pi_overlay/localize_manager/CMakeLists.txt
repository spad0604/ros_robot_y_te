cmake_minimum_required(VERSION 3.0.2)

project(localize_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  map_server
  message_generation
)

add_service_files(
  FILES
  StartMapping.srv
  SaveMap.srv
  StartLocalization.srv
  Stop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  std_srvs
  map_server
  message_runtime
)

catkin_install_python(
  PROGRAMS
    scripts/localize_manager_node.py
    scripts/sensor_telemetry_uploader.py
    scripts/order_listener.py
    scripts/delivery_engine.py
    scripts/delivery_executor.py
    scripts/teleop_bridge.py
    scripts/map_uploader.py
    scripts/vision_alert_detector.py
    scripts/follow_me_bridge.py
    scripts/follow_me_motion.py
    scripts/follow_person_pose.py
    scripts/robot_alert_uploader.py
    scripts/robot_pose_uploader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/battery_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/vision_python.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
