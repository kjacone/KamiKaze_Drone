cmake_minimum_required(VERSION 3.0.2)
project(drone_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  message_generation
)

# Declare custom messages
add_message_files(
  FILES
  BBox.msg
  Object.msg
  DetectedObjects.msg
  Target.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Catkin package
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
)

# Install all Python scripts
catkin_install_python(PROGRAMS
  scripts/vehicle_state_monitor.py
  scripts/target_tracking_controller.py
  scripts/yolo_detector.py
  scripts/camera_calibration.py
  scripts/sensor_fusion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files and config
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)