cmake_minimum_required(VERSION 3.16)
project(social_nav_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(social_nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xtensor REQUIRED)

set(dependencies rclcpp pluginlib nav2_costmap_2d nav2_mppi_controller social_nav_msgs tf2 tf2_ros)

add_library(social_layer SHARED src/social_layer.cpp)
target_include_directories(social_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(social_layer ${dependencies})

# The Humble MPPI controller is built with xsimd-backed xtensor and AVX2 (32-byte aligned
# tensors); critics share xtensor types with it, so they must use the same configuration.
add_library(social_critics SHARED src/dynamic_human_critic.cpp src/social_speed_critic.cpp
  src/passing_side_critic.cpp)
target_include_directories(social_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_definitions(social_critics PRIVATE XTENSOR_USE_XSIMD)
target_compile_options(social_critics PRIVATE -O3 -mavx2 -mfma)
target_link_libraries(social_critics xtensor)
ament_target_dependencies(social_critics ${dependencies})

pluginlib_export_plugin_description_file(nav2_costmap_2d social_layer_plugin.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics_plugin.xml)

install(TARGETS social_layer social_critics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES social_layer_plugin.xml critics_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_social_model test/test_social_model.cpp)
  target_link_libraries(test_social_model social_layer)
  ament_target_dependencies(test_social_model ${dependencies})
  ament_add_gtest(test_passing_side test/test_passing_side.cpp)
  target_compile_definitions(test_passing_side PRIVATE XTENSOR_USE_XSIMD)
  target_compile_options(test_passing_side PRIVATE -mavx2 -mfma)
  target_link_libraries(test_passing_side social_critics)
  ament_target_dependencies(test_passing_side ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
