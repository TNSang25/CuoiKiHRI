cmake_minimum_required(VERSION 3.8)
project(human_gazebo)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIR})
link_directories(${GAZEBO_LIBRARY_DIRS} ${TINYXML2_LIBRARY_DIRS})

# Gazebo world plugin: animates each person (actor + walking legs) from the agent controller.
add_library(PeoplePlugin SHARED src/PeoplePlugin.cpp)
ament_target_dependencies(PeoplePlugin rclcpp gazebo gazebo_ros social_nav_msgs geometry_msgs
  tf2 tf2_geometry_msgs)
target_link_libraries(PeoplePlugin ${GAZEBO_LIBRARIES})

# Adds the people (actors, leg bodies, plugin) of a scenario to a base world file.
add_executable(world_generator src/WorldGenerator.cpp)
ament_target_dependencies(world_generator rclcpp social_nav_msgs geometry_msgs tf2 tf2_ros
  tf2_geometry_msgs tinyxml2_vendor)
target_link_libraries(world_generator ${TINYXML2_LIBRARY})

install(TARGETS world_generator DESTINATION lib/${PROJECT_NAME})
install(TARGETS PeoplePlugin ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gait_balance test/test_gait_balance.cpp)
  target_compile_definitions(test_gait_balance PRIVATE
    HUMAN_MODEL_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/models")
  target_link_libraries(test_gait_balance ${GAZEBO_LIBRARIES})
endif()

ament_package()
