// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "transform_buffer.hpp"

namespace tf_buffer_server_test
{
namespace
{
tf2_msgs::msg::TFMessage sample(double x)
{
  tf2_msgs::msg::TFMessage message;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "sensor";
  transform.header.stamp.sec = 10;
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.x = x;
  message.transforms.push_back(transform);
  return message;
}
}  // namespace

TEST(TransformBuffer, RestoresStaticAfterUnderlyingStorageIsLost)
{
  tf_buffer_server::TransformBuffer buffer(std::make_shared<rclcpp::Clock>());
  buffer.insert(sample(1.0), true);
  buffer.insert(sample(2.0), true);
  auto invalid = sample(3.0);
  invalid.transforms.front().transform.rotation.w = 0.0;
  buffer.insert(invalid, true);  // Rejected data must not replace the retained static TF.

  // Emulate an emptied native cache even on Humble/Jazzy, where native clear retains static TF.
  // Bypass insertion tracking and replace native storage with an ordinary dynamic cache.
  auto & native = static_cast<tf2::BufferCore &>(buffer);
  ASSERT_TRUE(native.setTransform(sample(99.0).transforms.front(), "test", false));
  for (int i = 0; i < 3; ++i) {
    buffer.clear();
    auto result = buffer.lookupTransform("map", "sensor", tf2::TimePointZero);
    EXPECT_DOUBLE_EQ(result.transform.translation.x, 2.0);
  }
}

TEST(TransformBuffer, DynamicReplacementIsNotRestoredAsStatic)
{
  tf_buffer_server::TransformBuffer buffer(std::make_shared<rclcpp::Clock>());
  buffer.insert(sample(1.0), true);
  buffer.insert(sample(2.0), false);
  buffer.clear();
  EXPECT_THROW(buffer.lookupTransform("map", "sensor", tf2::TimePointZero),
      tf2::TransformException);
}
}  // namespace tf_buffer_server_test
