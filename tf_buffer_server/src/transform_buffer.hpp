// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER__TRANSFORM_BUFFER_HPP_
#define TF_BUFFER_SERVER__TRANSFORM_BUFFER_HPP_

#include <map>
#include <mutex>
#include <tf2_ros/buffer.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace tf_buffer_server
{
/// Keep accepted static transforms so clear has identical semantics across ROS distributions.
class TransformBuffer : public tf2_ros::Buffer
{
public:
  using tf2_ros::Buffer::Buffer;
  void insert(const tf2_msgs::msg::TFMessage & message, bool is_static);
  void clear() override;

private:
  // Serialize insertion and clearing, including clears triggered by ROS clock jumps.
  std::mutex cache_mutex_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> static_transforms_;
};
}  // namespace tf_buffer_server
#endif
