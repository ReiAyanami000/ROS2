cmake_minimum_required(VERSION 3.8)
project(transport_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteMotion.action"
  "action/ExecuteRoute.action"
  "msg/BaseStatus.msg"
  "msg/ConnectionStatus.msg"
  "srv/StopBase.srv"
  "msg/RouteSegment.msg"
  "msg/ProcessedRoute.msg"
  DEPENDENCIES action_msgs std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
