// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "tf_buffer_server/components.hpp"
#include "server.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <rclcpp_components/register_node_macro.hpp>

namespace tf_buffer_server
{
namespace
{
template<typename NodeT>
Settings declare_settings(NodeT & node)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Fixed at node creation";
  const double cache = node.declare_parameter("buffer.cache_time_seconds", 10.0, descriptor);
  const double timeout = node.declare_parameter("requests.max_timeout_seconds", 5.0, descriptor);
  const auto pending = node.declare_parameter("requests.max_pending", 256, descriptor);
  const auto depth = node.declare_parameter("subscriptions.tf_depth", 100, descriptor);
  const auto static_depth = node.declare_parameter("subscriptions.tf_static_depth", 100,
        descriptor);
  const bool spin_thread = node.declare_parameter("listener.spin_thread", false, descriptor);
  if (!std::isfinite(cache) || cache <= 0.0 || cache > 1e9 ||
    !std::isfinite(timeout) || timeout < 0.0 || timeout > 1e9 || pending <= 0 ||
    depth <= 0 || static_depth <= 0 || depth > std::numeric_limits<int>::max() ||
    static_depth > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument("Invalid TF buffer settings: durations must be finite and <= 1e9 "
            "seconds, cache time and counts positive, maximum timeout nonnegative");
  }
  return {cache, timeout, static_cast<size_t>(pending),
    static_cast<size_t>(depth), static_cast<size_t>(static_depth), spin_thread};
}

template<typename NodeT>
std::shared_ptr<Server> make_server(NodeT & node, const Settings & settings)
{
  auto server = std::make_shared<Server>(node.get_clock(), node.get_node_base_interface(),
    node.get_node_timers_interface(), settings, node.get_logger());
  server->initialize();
  return server;
}
}  // namespace

BufferServer::BufferServer(const rclcpp::NodeOptions & options)
: Node("tf_buffer_server", options)
{
  server_ = make_server(*this, declare_settings(*this));
  try {
    server_->start(this);
  } catch (...) {
    server_->stop(Status::INTERNAL_ERROR, "Server startup failed");
    throw;
  }
}
BufferServer::~BufferServer()
{
  // Keep Server owned here while joining any in-flight listener callback.
  server_->stop(Status::SHUTTING_DOWN, "Server shutting down");
}

LifecycleBufferServer::LifecycleBufferServer(const rclcpp::NodeOptions & options)
: LifecycleNode("tf_buffer_server", options),
  settings_(std::make_unique<Settings>(declare_settings(*this))) {}
LifecycleBufferServer::~LifecycleBufferServer()
{
  if (server_) {
    server_->stop(Status::SHUTTING_DOWN, "Server shutting down");
  }
}

LifecycleBufferServer::CallbackReturn LifecycleBufferServer::on_configure(
  const rclcpp_lifecycle::State &)
{
  server_ = make_server(*this, *settings_);
  return CallbackReturn::SUCCESS;
}
LifecycleBufferServer::CallbackReturn LifecycleBufferServer::on_activate(
  const rclcpp_lifecycle::State &)
{
  server_->start(this);
  return CallbackReturn::SUCCESS;
}
LifecycleBufferServer::CallbackReturn LifecycleBufferServer::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  server_->stop();
  return CallbackReturn::SUCCESS;
}
LifecycleBufferServer::CallbackReturn LifecycleBufferServer::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  server_.reset();
  return CallbackReturn::SUCCESS;
}
LifecycleBufferServer::CallbackReturn LifecycleBufferServer::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  if (server_) {
    server_->stop(Status::SHUTTING_DOWN, "Server shutting down");
    server_.reset();
  }
  return CallbackReturn::SUCCESS;
}
LifecycleBufferServer::CallbackReturn LifecycleBufferServer::on_error(
  const rclcpp_lifecycle::State &)
{
  if (server_) {
    server_->stop(Status::INTERNAL_ERROR, "Lifecycle transition failed");
    server_.reset();
  }
  return CallbackReturn::SUCCESS;
}
}  // namespace tf_buffer_server

RCLCPP_COMPONENTS_REGISTER_NODE(tf_buffer_server::BufferServer)
RCLCPP_COMPONENTS_REGISTER_NODE(tf_buffer_server::LifecycleBufferServer)
