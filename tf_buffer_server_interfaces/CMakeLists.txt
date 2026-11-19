# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(tf_buffer_server_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BufferStatus.msg"
  "srv/GetFrames.srv"
  "srv/LookupTransform.srv"
  "srv/LookupTransformFull.srv"
  "srv/CanTransform.srv"
  "srv/CanTransformFull.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
