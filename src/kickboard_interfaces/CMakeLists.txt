cmake_minimum_required(VERSION 3.8)
project(kickboard_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScooterState.msg"
  "msg/RobotStatus.msg"
  "msg/QueueItem.msg"
  "msg/QueueStatus.msg"
  "msg/AdminAlert.msg"
  "msg/TagDetection.msg"
  "msg/TagDetectionArray.msg"
  "msg/ScooterTruthArray.msg"
  "srv/RequestCollection.srv"
  "srv/GetQueue.srv"
  "srv/CancelRequest.srv"
  "srv/SetConfig.srv"
  "srv/EnqueueRequest.srv"
  "srv/SelectNextTarget.srv"
  "srv/UpdateRequestStatus.srv"
  "action/CollectScooter.action"
  "action/DockToCharger.action"
  "action/GoToPose.action"
  "action/LoadScooter.action"
  "action/UnloadScooter.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
