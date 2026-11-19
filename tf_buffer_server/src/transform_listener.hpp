// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER__TRANSFORM_LISTENER_HPP_
#define TF_BUFFER_SERVER__TRANSFORM_LISTENER_HPP_

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/qos.hpp>
#include "transform_buffer.hpp"

namespace tf_buffer_server
{
/// Standard TF subscriptions feeding the buffer's static-preserving insertion path.
class TransformListener
{
public:
  template<typename NodeT>
  TransformListener(
    TransformBuffer & buffer, NodeT * node, bool spin_thread,
    size_t tf_depth, size_t static_depth)
  {
    rclcpp::SubscriptionOptions options;
    if (spin_thread) {
      group_ = node->get_node_base_interface()->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
      options.callback_group = group_;
    }
    dynamic_ = node->template create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf2_ros::DynamicListenerQoS(tf_depth),
      [&buffer](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        buffer.insert(*message, false);
      }, options);
    static_ = node->template create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticListenerQoS(static_depth),
      [&buffer](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        buffer.insert(*message, true);
      }, options);
    if (spin_thread) {
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(group_, node->get_node_base_interface());
      thread_ = std::thread([this]() {executor_->spin();});
    }
  }

  ~TransformListener()
  {
    if (executor_) {
      executor_->cancel();
      thread_.join();
    }
  }

private:
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_, static_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};
}  // namespace tf_buffer_server
#endif
