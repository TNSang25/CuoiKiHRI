cmake_minimum_required(VERSION 3.8)
project(bot_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(bot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include)
include_directories(SYSTEM
  #${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${TINYXML2_LIBRARY_DIRS}
  #${TINYXML2_LIBRARY_PATH}
)


add_library(BotPeoplePlugin SHARED src/BotPeoplePlugin.cpp)
target_include_directories(BotPeoplePlugin PUBLIC include) #${ignition-common3_INCLUDE_DIRS}
ament_target_dependencies(BotPeoplePlugin 
  rclcpp 
  gazebo 
  gazebo_ros 
  bot_interfaces 
  geometry_msgs
  tf2 
  tf2_geometry_msgs
) 
ament_export_libraries(BotPeoplePlugin)
target_link_libraries(BotPeoplePlugin ${GAZEBO_LIBRARIES}) #${ignition-common3_LIBRARIES}


add_executable(bot_world_generator src/WorldGenerator.cpp)
ament_target_dependencies(bot_world_generator rclcpp bot_interfaces geometry_msgs tf2_geometry_msgs tf2 tf2_ros tinyxml2_vendor)
target_link_libraries(bot_world_generator ${TINYXML2_LIBRARY})

install(TARGETS bot_world_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS BotPeoplePlugin 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY worlds/
   DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY maps/
   DESTINATION share/${PROJECT_NAME}/maps
)

install(DIRECTORY scenarios/
   DESTINATION share/${PROJECT_NAME}/scenarios
)

install(DIRECTORY behavior_trees/
   DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY media/models
   DESTINATION share/${PROJECT_NAME}/
)
# install(DIRECTORY media/models/textures
#    DESTINATION share/${PROJECT_NAME}/textures
# )

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gait_balance test/test_gait_balance.cpp)
  target_compile_definitions(test_gait_balance PRIVATE
    BOT_MODEL_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/media/models")
  target_link_libraries(test_gait_balance ${GAZEBO_LIBRARIES})
endif()

ament_package()
