cmake_minimum_required(VERSION 3.16)
project(tf_buffer_server_client)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
set(dependencies rclcpp geometry_msgs builtin_interfaces tf_buffer_server_interfaces tf2_msgs std_srvs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/client.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# Humble's client factory takes an rmw profile; Jazzy and newer accept QoS objects.
if(rclcpp_VERSION VERSION_LESS "28.0.0")
  target_compile_definitions(${PROJECT_NAME} PRIVATE TF_BUFFER_CLIENT_LEGACY_QOS)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC rclcpp::rclcpp ${geometry_msgs_TARGETS}
  ${builtin_interfaces_TARGETS} ${tf_buffer_server_interfaces_TARGETS} ${tf2_msgs_TARGETS} ${std_srvs_TARGETS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ament_python_install_package(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  ament_add_gtest(test_client test/test_client.cpp TIMEOUT 60)
  target_link_libraries(test_client ${PROJECT_NAME} rclcpp_lifecycle::rclcpp_lifecycle)
  ament_add_pytest_test(test_python_client test/test_client.py TIMEOUT 60)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
