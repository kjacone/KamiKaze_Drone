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
catkin_python_setup()
# Declare custom messages
add_message_files(
  FILES
  BBox.msg
  Object.msg
  DetectedObjects.msg
  Target.msg
  TrackedTarget.msg
  TrackedTargets.msg
  ControlCommand.msg
  MissionStatus.msg
  Command.msg
  CommandResponse.msg
  SafetyStatus.msg
  NodeHealth.msg
  DiagnosticStatus.msg
)

# Declare services
add_service_files(
  FILES
  ValidateParameter.srv
  ReloadConfig.srv
  EmergencyStop.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
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
  # Core nodes
  scripts/core/camera_calibration.py
  scripts/core/sensor_fusion.py
  scripts/core/obstacle_avoidance_lib.py
  
  # Detectors
  scripts/detectors/yolo_detector.py
  
  # Controllers
  scripts/controllers/mission_manager.py
  scripts/controllers/command_interpreter.py
  scripts/controllers/target_tracking_controller.py
  scripts/controllers/performance_monitor.py
  
  # Monitors
  scripts/monitors/vehicle_state_monitor.py
  scripts/monitors/safety_monitor.py
  scripts/monitors/system_health_monitor.py
  scripts/monitors/health_checker.py
  
  # Utils
  src/drone_control/utils/error_handler.py
  src/drone_control/utils/message_validator.py
  src/drone_control/utils/parameter_loader.py
  src/drone_control/utils/logging_framework.py
  src/drone_control/utils/metrics_collector.py
  src/drone_control/utils/neural_planner.py
  src/drone_control/utils/parameter_validator.py
  src/drone_control/utils/reload_parameters.py
  src/drone_control/utils/generate_schema_doc.py
  src/drone_control/utils/tracing.py
  src/drone_control/utils/ros_tracing.py
  src/drone_control/utils/generate_schema_doc.py
  src/drone_control/utils/correlation.py

  # lib
  src/drone_control/lib/control_lib.py
  src/drone_control/lib/mission_lib.py
  src/drone_control/lib/safety_lib.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files and config
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)