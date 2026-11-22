cmake_minimum_required(VERSION 3.16)
project(people_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Algorithm library (no ROS): clustering, leg detection, pairing, Kalman tracking, map mask.
add_library(leg_tracking SHARED src/tracking.cpp)
target_include_directories(leg_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(leg_tracking Eigen3::Eigen)

add_executable(leg_tracker src/leg_tracker_node.cpp)
target_link_libraries(leg_tracker leg_tracking)
ament_target_dependencies(leg_tracker rclcpp social_nav_msgs sensor_msgs nav_msgs geometry_msgs
  visualization_msgs tf2 tf2_ros)

add_executable(people_adapter src/people_adapter.cpp)
ament_target_dependencies(people_adapter rclcpp social_nav_msgs)

install(TARGETS leg_tracking ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS leg_tracker people_adapter DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tracker_eval.py DESTINATION lib/${PROJECT_NAME} RENAME tracker_eval)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tracking test/test_tracking.cpp)
  target_link_libraries(test_tracking leg_tracking)
endif()

ament_package()
