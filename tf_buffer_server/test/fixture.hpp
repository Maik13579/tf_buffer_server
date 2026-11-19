// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER_TEST__FIXTURE_HPP_
#define TF_BUFFER_SERVER_TEST__FIXTURE_HPP_
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <tuple>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "tf_buffer_server/components.hpp"
#include <tf_buffer_server_interfaces/msg/buffer_status.hpp>
#include <tf_buffer_server_interfaces/srv/get_frames.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform_full.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform_full.hpp>
#include <tf2_msgs/srv/frame_graph.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace tf_buffer_server_test
{
using namespace std::chrono_literals;
namespace srv = tf_buffer_server_interfaces::srv;
using Status = tf_buffer_server_interfaces::msg::BufferStatus;

class ServerTest : public ::testing::TestWithParam<std::tuple<bool, bool>>
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void start(bool lifecycle = false, bool sim_time = false)
  {
    static std::atomic<unsigned> id{0};
    ns_ = "/tf_server_test_" + std::to_string(++id);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns_,
        "-r", "/tf:=" + ns_ + "/tf", "-r", "/tf_static:=" + ns_ + "/tf_static"});
    helper_ = std::make_shared<rclcpp::Node>("client", options);
    options.append_parameter_override("requests.max_pending", 2);
    options.append_parameter_override("requests.max_timeout_seconds", 2.0);
    options.append_parameter_override("buffer.cache_time_seconds", 2.0);
    options.append_parameter_override("use_sim_time", sim_time);
    options.append_parameter_override("listener.spin_thread", std::get<1>(GetParam()));
    if (std::get<0>(GetParam())) {
      executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 3);
    } else {
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    }
    executor_->add_node(helper_);
    if (lifecycle) {
      lifecycle_ = std::make_shared<tf_buffer_server::LifecycleBufferServer>(options);
      base_ = lifecycle_->get_node_base_interface();
    } else {
      ordinary_ = std::make_shared<tf_buffer_server::BufferServer>(options);
      base_ = ordinary_->get_node_base_interface();
    }
    executor_->add_node(base_);
    thread_ = std::thread([this]() {executor_->spin();});
    tf_ = helper_->create_publisher<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS(100));
    static_ = helper_->create_publisher<tf2_msgs::msg::TFMessage>(
      "tf_static", rclcpp::QoS(100).transient_local());
  }
  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (thread_.joinable()) {thread_.join();}
    executor_.reset();
    base_.reset();
    lifecycle_.reset();
    ordinary_.reset();
    helper_.reset();
  }
  template<typename Predicate>
  void wait(Predicate predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!predicate()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Condition timed out");
      }
      std::this_thread::sleep_for(5ms);
    }
  }
  std::string endpoint(const std::string & name) const {return ns_ + "/tf_buffer_server/" + name;}
  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr client(const std::string & name)
  {
    auto result = helper_->create_client<ServiceT>(endpoint(name));
    if (!result->wait_for_service(5s)) {throw std::runtime_error("Service not available: " + name);}
    return result;
  }
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const std::string & name,
    const typename ServiceT::Request & request = typename ServiceT::Request())
  {
    auto connection = client<ServiceT>(name);
    auto future =
      connection->async_send_request(std::make_shared<typename ServiceT::Request>(request));
    if (future.wait_for(5s) != std::future_status::ready) {
      throw std::runtime_error("Response timed out: " + name);
    }
    return future.get();
  }
  void transition(uint8_t id)
  {
    lifecycle_msgs::srv::ChangeState::Request request;
    request.transition.id = id;
    ASSERT_TRUE(call<lifecycle_msgs::srv::ChangeState>("change_state", request)->success);
  }
  geometry_msgs::msg::TransformStamped transform(
    const std::string & parent, const std::string & child, int seconds, double x)
  {
    geometry_msgs::msg::TransformStamped result;
    result.header.frame_id = parent;
    result.child_frame_id = child;
    result.header.stamp.sec = seconds;
    result.transform.translation.x = x;
    result.transform.rotation.w = 1.0;
    return result;
  }
  void publish(const geometry_msgs::msg::TransformStamped & transform, bool is_static = false)
  {
    auto publisher = is_static ? static_ : tf_;
    wait([&]() {return publisher->get_subscription_count() > 0;});
    tf2_msgs::msg::TFMessage message;
    message.transforms.push_back(transform);
    publisher->publish(message);
  }
  srv::LookupTransform::Request query(std::string source = "base")
  {
    srv::LookupTransform::Request result;
    result.target_frame = "map";
    result.source_frame = std::move(source);
    return result;
  }
  void wait_available(const srv::LookupTransform::Request & request)
  {
    wait([&]() {
        return call<srv::LookupTransform>("lookup_transform", request)->status.code == Status::OK;
    });
  }
  std::string ns_;
  rclcpp::Node::SharedPtr helper_;
  std::shared_ptr<tf_buffer_server::BufferServer> ordinary_;
  std::shared_ptr<tf_buffer_server::LifecycleBufferServer> lifecycle_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_;
  std::unique_ptr<rclcpp::Executor> executor_;
  std::thread thread_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_, static_;
};
}  // namespace tf_buffer_server_test
#endif
