cmake_minimum_required(VERSION 3.16)
project(social_nav_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Agent.msg"
  "msg/AgentBehavior.msg"
  "msg/Agents.msg"
  "msg/People.msg"
  "msg/Person.msg"
  "srv/ComputeAgent.srv"
  "srv/ComputeAgents.srv"
  "srv/GetAgents.srv"
  "srv/GetParameters.srv"
  "srv/MoveAgent.srv"
  "srv/ResetAgents.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
