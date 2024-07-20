# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(vehicle_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ActuationStatus.msg"
    "msg/ActuationStatusStamped.msg"
    "msg/BatteryStatus.msg"
    "msg/ControlModeReport.msg"
    "msg/DoorStatus.msg"
    "msg/GearReport.msg"
    "msg/HazardLightsReport.msg"
    "msg/Shift.msg"
    "msg/ShiftStamped.msg"
    "msg/Steering.msg"
    "msg/SteeringReport.msg"
    "msg/SteeringWheelStatusStamped.msg"
    "msg/TurnIndicatorsReport.msg"
    "msg/TurnSignal.msg"
    "msg/VehicleEmergencyStamped.msg"
    "msg/VehicleOdometry.msg"
    "msg/VelocityReport.msg"
  DEPENDENCIES
    "builtin_interfaces"
    "std_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
