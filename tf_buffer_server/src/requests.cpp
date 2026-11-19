// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "server.hpp"
#include <stdexcept>

namespace tf_buffer_server
{
namespace
{
void validate_frame(const std::string & frame)
{
  if (frame.empty() || frame.front() == '/') {
    throw std::invalid_argument("Frame IDs must be nonempty and must not start with '/'");
  }
}

tf2::TimePoint time_from(const builtin_interfaces::msg::Time & time)
{
  if (time.sec < 0 || time.nanosec >= 1000000000u) {
    throw std::invalid_argument("Time must have nonnegative seconds and nanoseconds < 1e9");
  }
  return tf2::TimePoint(std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nanosec));
}

tf2::Duration duration_from(const builtin_interfaces::msg::Duration & duration)
{
  if (duration.sec < 0 || duration.nanosec >= 1000000000u) {
    throw std::invalid_argument("Timeout must have nonnegative seconds and nanoseconds < 1e9");
  }
  return std::chrono::seconds(duration.sec) + std::chrono::nanoseconds(duration.nanosec);
}

template<typename RequestT>
Query basic_query(const RequestT & request)
{
  validate_frame(request.target_frame);
  validate_frame(request.source_frame);
  return {request.target_frame, request.source_frame, "", time_from(request.time),
    time_from(request.time), duration_from(request.timeout)};
}

template<typename RequestT>
Query full_query(const RequestT & request)
{
  validate_frame(request.target_frame);
  validate_frame(request.source_frame);
  validate_frame(request.fixed_frame);
  return {request.target_frame, request.source_frame, request.fixed_frame,
    time_from(request.target_time), time_from(request.source_time), duration_from(request.timeout)};
}
}  // namespace

Query query_from(const srv::LookupTransform::Request & request) {return basic_query(request);}
Query query_from(const srv::CanTransform::Request & request) {return basic_query(request);}
Query query_from(const srv::LookupTransformFull::Request & request) {return full_query(request);}
Query query_from(const srv::CanTransformFull::Request & request) {return full_query(request);}

void set_result(
  srv::LookupTransform::Response & response,
  const geometry_msgs::msg::TransformStamped & transform) {response.transform = transform;}
void set_result(
  srv::LookupTransformFull::Response & response,
  const geometry_msgs::msg::TransformStamped & transform) {response.transform = transform;}
void set_result(
  srv::CanTransform::Response & response,
  const geometry_msgs::msg::TransformStamped &) {response.can_transform = true;}
void set_result(
  srv::CanTransformFull::Response & response,
  const geometry_msgs::msg::TransformStamped &) {response.can_transform = true;}

Status status(uint8_t code, const std::string & message)
{
  Status result;
  result.code = code;
  result.message = message;
  return result;
}

Status exception_status(const std::exception & error)
{
  uint8_t code = Status::INTERNAL_ERROR;
  if (dynamic_cast<const tf2::TimeoutException *>(&error)) {
    code = Status::TIMEOUT;
  } else if (dynamic_cast<const tf2::InvalidArgumentException *>(&error) ||
    dynamic_cast<const std::invalid_argument *>(&error))
  {
    code = Status::INVALID_ARGUMENT;
  } else if (dynamic_cast<const tf2::ConnectivityException *>(&error)) {
    code = Status::CONNECTIVITY_ERROR;
  } else if (dynamic_cast<const tf2::ExtrapolationException *>(&error)) {
    code = Status::EXTRAPOLATION_ERROR;
  } else if (dynamic_cast<const tf2::LookupException *>(&error)) {
    code = Status::LOOKUP_ERROR;
  } else if (dynamic_cast<const tf2::TransformException *>(&error)) {
    code = Status::TRANSFORM_ERROR;
  }
  return status(code, error.what());
}
}  // namespace tf_buffer_server
