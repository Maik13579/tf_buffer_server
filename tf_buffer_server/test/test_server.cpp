// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "fixture.hpp"
#include <algorithm>
#include <limits>

namespace tf_buffer_server_test
{
TEST_P(ServerTest, LookupInterpolationGraphAndClear)
{
  start();
  publish(transform("map", "base", 10, 1));
  publish(transform("map", "base", 12, 3));
  publish(transform("base", "laser", 0, 2), true);
  wait_available(query("laser"));
  auto request = query();
  request.time.sec = 11;
  auto result = call<srv::LookupTransform>("lookup_transform", request);
  ASSERT_EQ(result->status.code, Status::OK);
  EXPECT_DOUBLE_EQ(result->transform.transform.translation.x, 2.0);
  EXPECT_EQ(result->transform.header.stamp.sec, 11);
  auto latest = call<srv::LookupTransform>("lookup_transform", query());
  EXPECT_DOUBLE_EQ(latest->transform.transform.translation.x, 3.0);
  auto frames = call<srv::GetFrames>("get_frames")->frames;
  EXPECT_EQ(frames, (std::vector<std::string>{"base", "laser", "map"}));
  EXPECT_NE(call<tf2_msgs::srv::FrameGraph>("get_frame_graph")->frame_yaml.find("laser"),
      std::string::npos);
  EXPECT_TRUE(call<std_srvs::srv::Trigger>("clear")->success);
  EXPECT_NE(call<srv::LookupTransform>("lookup_transform", query())->status.code, Status::OK);
  request.target_frame = "base";
  request.source_frame = "laser";
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code, Status::OK);
  EXPECT_EQ(call<srv::GetFrames>("get_frames")->frames, frames);
}

TEST_P(ServerTest, FullLookupUsesSeparateTimes)
{
  start();
  publish(transform("map", "base", 10, 1));
  publish(transform("map", "base", 12, 3));
  wait_available(query());
  srv::LookupTransformFull::Request request;
  request.target_frame = "base";
  request.target_time.sec = 12;
  request.source_frame = "base";
  request.source_time.sec = 10;
  request.fixed_frame = "map";
  auto result = call<srv::LookupTransformFull>("lookup_transform_full", request);
  ASSERT_EQ(result->status.code, Status::OK);
  EXPECT_DOUBLE_EQ(result->transform.transform.translation.x, -2.0);
  srv::CanTransformFull::Request check;
  check.target_frame = request.target_frame;
  check.target_time = request.target_time;
  check.source_frame = request.source_frame;
  check.source_time = request.source_time;
  check.fixed_frame = request.fixed_frame;
  EXPECT_TRUE(call<srv::CanTransformFull>("can_transform_full", check)->can_transform);
}

TEST_P(ServerTest, DeferredLookupDoesNotBlockAndClearCancels)
{
  start();
  auto connection = client<srv::LookupTransform>("lookup_transform");
  auto request = query();
  request.timeout.sec = 1;
  auto future =
    connection->async_send_request(std::make_shared<srv::LookupTransform::Request>(request));
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(future.wait_for(0ms), std::future_status::timeout);
  EXPECT_EQ(call<srv::GetFrames>("get_frames")->status.code, Status::OK);
  publish(transform("map", "base", 10, 4));
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(future.get()->status.code, Status::OK);
  request.source_frame = "missing";
  auto canceled =
    connection->async_send_request(std::make_shared<srv::LookupTransform::Request>(request));
  std::this_thread::sleep_for(30ms);
  EXPECT_TRUE(call<std_srvs::srv::Trigger>("clear")->success);
  ASSERT_EQ(canceled.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(canceled.get()->status.code, Status::CLEARED);
  // Repeated cancellation must release TF waits and request capacity.
  for (int i = 0; i < 10; ++i) {
    auto pending =
      connection->async_send_request(std::make_shared<srv::LookupTransform::Request>(request));
    std::this_thread::sleep_for(10ms);
    call<std_srvs::srv::Trigger>("clear");
    ASSERT_EQ(pending.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(pending.get()->status.code, Status::CLEARED);
  }
}

TEST_P(ServerTest, FullDeferredWaitAndAvailability)
{
  start();
  publish(transform("map", "base", 10, 1));
  wait_available(query());
  srv::LookupTransformFull::Request request;
  request.target_frame = "target";
  request.target_time.sec = 10;
  request.source_frame = "base";
  request.source_time.sec = 10;
  request.fixed_frame = "map";
  request.timeout.sec = 1;
  auto connection = client<srv::LookupTransformFull>("lookup_transform_full");
  auto future =
    connection->async_send_request(std::make_shared<srv::LookupTransformFull::Request>(request));
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(call<srv::GetFrames>("get_frames")->status.code, Status::OK);
  publish(transform("map", "target", 10, 3));
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  auto response = future.get();
  ASSERT_EQ(response->status.code, Status::OK) << response->status.message;
  EXPECT_DOUBLE_EQ(response->transform.transform.translation.x, -2.0);
  srv::CanTransform::Request check;
  check.target_frame = "map";
  check.source_frame = "new_frame";
  check.timeout.sec = 1;
  auto availability = client<srv::CanTransform>("can_transform");
  auto available =
    availability->async_send_request(std::make_shared<srv::CanTransform::Request>(check));
  publish(transform("map", "new_frame", 10, 5));
  ASSERT_EQ(available.wait_for(3s), std::future_status::ready);
  EXPECT_TRUE(available.get()->can_transform);
}

TEST_P(ServerTest, ValidationErrorsAndCacheEviction)
{
  start();
  auto request = query();
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code,
      Status::LOOKUP_ERROR);
  for (const auto & frame : {std::string(""), std::string("/base")}) {
    request.source_frame = frame;
    EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code,
        Status::INVALID_ARGUMENT);
  }
  request = query();
  request.timeout.sec = -1;
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code,
      Status::INVALID_ARGUMENT);
  request.timeout.sec = 3;
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code,
      Status::INVALID_ARGUMENT);
  request.timeout.sec = 0;
  request.time.nanosec = 1000000000u;
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code,
      Status::INVALID_ARGUMENT);
  publish(transform("map", "base", 10, 1));
  publish(transform("map", "base", 14, 5));
  publish(transform("map", "base", 15, 6));
  publish(transform("other", "island", 14, 1));
  wait([&]() {
      return call<srv::LookupTransform>("lookup_transform",
        query())->transform.header.stamp.sec == 15;
  });
  request = query();
  request.time.sec = 10;
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code,
      Status::EXTRAPOLATION_ERROR);
  wait([&]() {return call<srv::GetFrames>("get_frames")->frames.size() == 4;});
  request = query("island");
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code,
      Status::CONNECTIVITY_ERROR);
}

TEST_P(ServerTest, PendingLimitAndTimeoutWithPausedSimulation)
{
  start(false, true);
  auto connection = client<srv::LookupTransform>("lookup_transform");
  auto request = query();
  request.timeout.nanosec = 300000000u;
  auto first =
    connection->async_send_request(std::make_shared<srv::LookupTransform::Request>(request));
  auto second =
    connection->async_send_request(std::make_shared<srv::LookupTransform::Request>(request));
  std::this_thread::sleep_for(40ms);
  auto third = call<srv::LookupTransform>("lookup_transform", request);
  EXPECT_EQ(third->status.code, Status::BUSY);
  ASSERT_EQ(first.wait_for(3s), std::future_status::ready);
  ASSERT_EQ(second.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(first.get()->status.code, Status::TIMEOUT);
  EXPECT_EQ(second.get()->status.code, Status::TIMEOUT);
  srv::CanTransform::Request check;
  check.target_frame = "map";
  check.source_frame = "base";
  check.timeout.nanosec = 50000000u;
  auto availability = call<srv::CanTransform>("can_transform", check);
  EXPECT_FALSE(availability->can_transform);
  EXPECT_EQ(availability->status.code, Status::TIMEOUT);
}

TEST_P(ServerTest, ClockResetCancelsPending)
{
  start(false, true);
  auto clock = helper_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
  wait([&]() {return clock->get_subscription_count() > 0;});
  rosgraph_msgs::msg::Clock message;
  message.clock.sec = 20;
  clock->publish(message);
  wait([&]() {return ordinary_->now().seconds() == 20.0;});
  call<srv::GetFrames>("get_frames");
  auto connection = client<srv::LookupTransform>("lookup_transform");
  auto request = query();
  request.timeout.sec = 1;
  auto future =
    connection->async_send_request(std::make_shared<srv::LookupTransform::Request>(request));
  std::this_thread::sleep_for(30ms);
  message.clock.sec = 10;
  clock->publish(message);
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(future.get()->status.code, Status::CLOCK_RESET);
}

TEST(Settings, RejectInvalidParameters)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.append_parameter_override("buffer.cache_time_seconds",
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(tf_buffer_server::BufferServer node(options), std::invalid_argument);
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(Executors, ServerTest,
    ::testing::Combine(::testing::Values(false, true), ::testing::Values(false, true)));
}  // namespace tf_buffer_server_test
