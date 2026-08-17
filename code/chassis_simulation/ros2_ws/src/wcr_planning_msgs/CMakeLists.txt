cmake_minimum_required(VERSION 3.8)
project(wcr_planning_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/ObstacleUpdate.msg"
  "msg/TrajectoryPoint.msg"
  "msg/TimedTrajectory.msg"
  "msg/TaskCommand.msg"
  "msg/ManualCommand.msg"
  "msg/TaskState.msg"
  "msg/DeviceCommand.msg"
  "msg/DeviceState.msg"
  "msg/VisionTiming.msg"
  "msg/InspectionState.msg"
  "msg/ProbeAlignment.msg"
  "msg/WeldSeamCandidate.msg"
  "msg/WeldSeamCandidateArray.msg"
  "msg/WeldSeamSelection.msg"
  "msg/MappedWeldSeam.msg"
  "msg/EnvironmentMap.msg"
  "msg/ProbeRailCommand.msg"
  "msg/ProbeRailState.msg"
  "msg/LinearMotorCommand.msg"
  "msg/InspectionContext.msg"
  "srv/GetFusionState.srv"
  "srv/GetInspectionContext.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
