// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER_CLIENT__CLIENT_HPP_
#define TF_BUFFER_SERVER_CLIENT__CLIENT_HPP_

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf_buffer_server_interfaces/msg/buffer_status.hpp>

namespace tf_buffer_server_client
{
using Status = tf_buffer_server_interfaces::msg::BufferStatus;
using Duration = std::chrono::nanoseconds;
using Transform = geometry_msgs::msg::TransformStamped;
using Frames = std::vector<std::string>;
template<typename T> using Callback = std::function<void(std::shared_future<T>)>;

enum class ErrorKind {SERVER, UNAVAILABLE, RESPONSE_TIMEOUT, CLOSED};
class ClientError : public std::runtime_error
{
public:
  ClientError(ErrorKind kind, const std::string & message, std::optional<Status> status = {})
  : std::runtime_error(message), kind(kind), status(std::move(status)) {}
  ErrorKind kind;
  std::optional<Status> status;
};

/// Interfaces work with both rclcpp::Node and rclcpp_lifecycle::LifecycleNode.
struct NodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers;
};
struct ClientOptions
{
  // Local response deadline = server wait duration + this real-time margin.
  Duration response_margin = std::chrono::seconds(2);
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

/// Borrow a node; its executor must run to deliver replies. No hidden spinning or threads.
/// Blocking methods require an executor running elsewhere; avoid single-threaded callbacks.
/// The node must outlive this object. Close completes pending calls and leaves the node running.
class Client
{
public:
  explicit Client(
    NodeInterfaces interfaces, std::string server = "tf_buffer_server",
    ClientOptions options = {});
  template<typename NodeT>
  explicit Client(NodeT * node, std::string server = "tf_buffer_server", ClientOptions options = {})
  : Client(
      NodeInterfaces{node->get_node_base_interface(), node->get_node_graph_interface(),
        node->get_node_services_interface(), node->get_node_timers_interface()}, server, options) {}
  template<typename NodeT>
  explicit Client(
    const std::shared_ptr<NodeT> & node, std::string server = "tf_buffer_server",
    ClientOptions options = {})
  : Client(node.get(), server, options) {}
  ~Client();
  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;

  bool is_ready() const;
  bool wait_for_server(Duration timeout = std::chrono::seconds(2)) const;
  void close();

  std::shared_future<Frames> get_frames_async(Callback<Frames> callback = {});
  Frames get_frames();
  std::shared_future<std::string> get_frame_graph_async(Callback<std::string> callback = {});
  std::string get_frame_graph();
  std::shared_future<void> clear_async(Callback<void> callback = {});
  void clear();

  std::shared_future<Transform> lookup_transform_async(
    const std::string & target,
    const std::string & source, rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME),
    Duration timeout = Duration::zero(), Callback<Transform> callback = {});
  Transform lookup_transform(
    const std::string & target, const std::string & source,
    rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME), Duration timeout = Duration::zero());
  std::shared_future<Transform> lookup_transform_full_async(
    const std::string & target,
    rclcpp::Time target_time, const std::string & source, rclcpp::Time source_time,
    const std::string & fixed, Duration timeout = Duration::zero(),
    Callback<Transform> callback = {});
  Transform lookup_transform_full(
    const std::string & target, rclcpp::Time target_time,
    const std::string & source, rclcpp::Time source_time, const std::string & fixed,
    Duration timeout = Duration::zero());

  std::shared_future<bool> can_transform_async(
    const std::string & target, const std::string & source,
    rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME), Duration timeout = Duration::zero(),
    Callback<bool> callback = {});
  bool can_transform(
    const std::string & target, const std::string & source,
    rclcpp::Time time = rclcpp::Time(0, 0, RCL_ROS_TIME), Duration timeout = Duration::zero());
  std::shared_future<bool> can_transform_full_async(
    const std::string & target,
    rclcpp::Time target_time, const std::string & source, rclcpp::Time source_time,
    const std::string & fixed, Duration timeout = Duration::zero(), Callback<bool> callback = {});
  bool can_transform_full(
    const std::string & target, rclcpp::Time target_time,
    const std::string & source, rclcpp::Time source_time, const std::string & fixed,
    Duration timeout = Duration::zero());

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tf_buffer_server_client
#endif
