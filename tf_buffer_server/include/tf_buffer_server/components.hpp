// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER__COMPONENTS_HPP_
#define TF_BUFFER_SERVER__COMPONENTS_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace tf_buffer_server
{
class Server;
struct Settings;

/// Ordinary component: starts listening and serving during construction.
class BufferServer : public rclcpp::Node
{
public:
  explicit BufferServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BufferServer() override;

private:
  std::shared_ptr<Server> server_;
};

/// Managed component: TF subscriptions and application services exist only while active.
class LifecycleBufferServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleBufferServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LifecycleBufferServer() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  std::unique_ptr<Settings> settings_;
  std::shared_ptr<Server> server_;
};
}  // namespace tf_buffer_server
#endif
