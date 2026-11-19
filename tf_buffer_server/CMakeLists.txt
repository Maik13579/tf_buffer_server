# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(tf_buffer_server)
find_package(ament_cmake REQUIRED)
set(dependencies rclcpp rclcpp_components rclcpp_lifecycle rcl_interfaces
  builtin_interfaces geometry_msgs std_srvs tf2 tf2_ros tf2_msgs tf_buffer_server_interfaces)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
add_library(${PROJECT_NAME} SHARED src/components.cpp src/server.cpp src/requests.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp rclcpp_components::component rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2 tf2_ros::tf2_ros ${rcl_interfaces_TARGETS} ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS} ${std_srvs_TARGETS} ${tf2_msgs_TARGETS}
  ${tf_buffer_server_interfaces_TARGETS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
rclcpp_components_register_nodes(${PROJECT_NAME}
  "tf_buffer_server::BufferServer" "tf_buffer_server::LifecycleBufferServer")
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_container.py TIMEOUT 90)
  find_package(rosgraph_msgs REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(test_server test/test_server.cpp test/test_lifecycle.cpp TIMEOUT 120)
  target_link_libraries(test_server ${PROJECT_NAME} ${rosgraph_msgs_TARGETS} ${lifecycle_msgs_TARGETS})
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
