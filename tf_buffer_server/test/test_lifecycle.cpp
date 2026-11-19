// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "fixture.hpp"
#include <lifecycle_msgs/msg/transition.hpp>

namespace tf_buffer_server_test
{
using LifecycleTest = ServerTest;
TEST_P(LifecycleTest, LifecycleReceptionServicesAndHistory)
{
  start(true);
  auto frames = helper_->create_client<srv::GetFrames>(endpoint("get_frames"));
  EXPECT_FALSE(frames->service_is_ready());
  EXPECT_EQ(tf_->get_subscription_count(), 0u);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_FALSE(frames->service_is_ready());
  EXPECT_EQ(tf_->get_subscription_count(), 0u);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  publish(transform("map", "base", 10, 4));
  publish(transform("base", "laser", 0, 1), true);
  wait_available(query("laser"));
  static_.reset();  // Reactivation must preserve static TF without rediscovery.
  auto connection = client<srv::LookupTransform>("lookup_transform");
  auto request = query("missing");
  request.timeout.sec = 1;
  auto pending =
    connection->async_send_request(std::make_shared<srv::LookupTransform::Request>(request));
  std::this_thread::sleep_for(30ms);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  ASSERT_EQ(pending.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(pending.get()->status.code, Status::INACTIVE);
  wait([&]() {return !frames->service_is_ready() && tf_->get_subscription_count() == 0;});
  transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  wait([&]() {return frames->service_is_ready();});
  EXPECT_NE(call<srv::LookupTransform>("lookup_transform", query())->status.code, Status::OK);
  request = query("laser");
  request.target_frame = "base";
  EXPECT_EQ(call<srv::LookupTransform>("lookup_transform", request)->status.code, Status::OK);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(call<srv::GetFrames>("get_frames")->status.code, Status::OK);
  transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  wait([&]() {return !frames->service_is_ready();});
}
INSTANTIATE_TEST_SUITE_P(ManagedExecutors, LifecycleTest,
    ::testing::Combine(::testing::Values(false, true), ::testing::Values(false, true)));
}  // namespace tf_buffer_server_test
