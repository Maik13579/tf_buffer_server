// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf_buffer_server_interfaces/srv/get_frames.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform_full.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform_full.hpp>
#include <tf2_msgs/srv/frame_graph.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "tf_buffer_server_client/client.hpp"

namespace
{
using namespace std::chrono_literals;
namespace srv = tf_buffer_server_interfaces::srv;
using tf_buffer_server_client::Client;
using tf_buffer_server_client::ClientError;
using tf_buffer_server_client::ErrorKind;
using Status = tf_buffer_server_client::Status;

class Clients : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    static std::atomic<unsigned> id{0};
    options_.arguments({"--ros-args", "-r", "__ns:=/tf_client_test_" + std::to_string(++id)});
    node_ = std::make_shared<rclcpp::Node>("client", options_);
    server_ = std::make_shared<rclcpp::Node>("server", options_);
    services_.push_back(server_->create_service<srv::GetFrames>("buffer/get_frames",
      [](srv::GetFrames::Request::SharedPtr, srv::GetFrames::Response::SharedPtr response) {
        response->frames = {"map", "base"};
      }));
    services_.push_back(server_->create_service<tf2_msgs::srv::FrameGraph>("buffer/get_frame_graph",
      [](tf2_msgs::srv::FrameGraph::Request::SharedPtr,
      tf2_msgs::srv::FrameGraph::Response::SharedPtr r) {
        r->frame_yaml = "base: {parent: map}";
      }));
    services_.push_back(server_->create_service<std_srvs::srv::Trigger>("buffer/clear",
      [](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr r) {
        r->success = true;
      }));
    services_.push_back(server_->create_service<srv::LookupTransform>("buffer/lookup_transform",
      [](std::shared_ptr<rclcpp::Service<srv::LookupTransform>> service,
      std::shared_ptr<rmw_request_id_t> header, srv::LookupTransform::Request::SharedPtr request) {
        if (request->source_frame == "never") {return;}
        srv::LookupTransform::Response response;
        response.status.code = code(request->source_frame);
        response.status.message = "lookup status";
        response.transform.header.frame_id = request->target_frame;
        response.transform.child_frame_id = request->source_frame;
        response.transform.header.stamp = request->time;
        response.transform.transform.translation.x = request->timeout.sec;
        service->send_response(*header, response);
      }));
    services_.push_back(server_->create_service<srv::LookupTransformFull>(
        "buffer/lookup_transform_full",
        [](srv::LookupTransformFull::Request::SharedPtr q,
        srv::LookupTransformFull::Response::SharedPtr r) {
          r->transform.header.frame_id = q->fixed_frame;
          r->transform.transform.translation.x = q->target_time.sec - q->source_time.sec;
      }));
    services_.push_back(server_->create_service<srv::CanTransform>("buffer/can_transform",
      [](srv::CanTransform::Request::SharedPtr q, srv::CanTransform::Response::SharedPtr r) {
        r->status.code = code(q->source_frame);
        r->can_transform = r->status.code == Status::OK;
      }));
    services_.push_back(server_->create_service<srv::CanTransformFull>("buffer/can_transform_full",
      [](srv::CanTransformFull::Request::SharedPtr q,
      srv::CanTransformFull::Response::SharedPtr r) {
        r->can_transform = q->fixed_frame == "map" && q->target_time.sec == 12 &&
        q->source_time.sec == 10;
      }));
    if (GetParam()) {
      executor_ =
        std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3);
    } else {executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();}
    client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_->add_node(node_);
    executor_->add_node(server_);
    tf_buffer_server_client::ClientOptions settings;
    settings.response_margin = 500ms;
    settings.callback_group = client_group_;
    client_ = std::make_unique<Client>(node_, "buffer", settings);
    thread_ = std::thread([this]() {executor_->spin();});
    ASSERT_TRUE(client_->wait_for_server(5s));
  }
  void TearDown() override
  {
    client_->close();
    executor_->cancel();
    thread_.join();
    timer_.reset();
    client_.reset();
    executor_.reset();
    services_.clear();
    node_.reset();
    server_.reset();
  }
  static uint8_t code(const std::string & frame)
  {
    return frame == "missing" ? Status::LOOKUP_ERROR : frame == "busy" ? Status::BUSY : Status::OK;
  }
  rclcpp::NodeOptions options_;
  rclcpp::Node::SharedPtr node_, server_;
  std::unique_ptr<Client> client_;
  std::unique_ptr<rclcpp::Executor> executor_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread thread_;
};

TEST_P(Clients, TypedValuesAndCallbacks)
{
  EXPECT_TRUE(client_->is_ready());
  EXPECT_EQ(client_->get_frames(), (std::vector<std::string>{"map", "base"}));
  EXPECT_EQ(client_->get_frame_graph(), "base: {parent: map}");
  client_->clear();
  auto value = client_->lookup_transform("map", "base", rclcpp::Time(10, 0), 1s);
  EXPECT_EQ(value.header.stamp.sec, 10);
  EXPECT_DOUBLE_EQ(value.transform.translation.x, 1.0);
  std::promise<std::string> callback;
  auto future = client_->lookup_transform_async("map", "base", rclcpp::Time(0, 0), 0ns,
      [&callback](std::shared_future<tf_buffer_server_client::Transform> response) {
        callback.set_value(response.get().header.frame_id);
    });
  auto notification = callback.get_future();
  ASSERT_EQ(notification.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(notification.get(), "map");
  EXPECT_EQ(future.get().child_frame_id, "base");
  auto full = client_->lookup_transform_full("base", rclcpp::Time(12, 0), "base",
      rclcpp::Time(10, 0), "map");
  EXPECT_DOUBLE_EQ(full.transform.translation.x, 2.0);
  EXPECT_TRUE(client_->can_transform_full("base", rclcpp::Time(12, 0), "base", rclcpp::Time(10, 0),
      "map"));
  EXPECT_TRUE(client_->can_transform("map", "base"));
}

TEST_P(Clients, CheckedErrorsAndAvailability)
{
  EXPECT_FALSE(client_->can_transform("map", "missing"));
  try {
    client_->lookup_transform("map", "busy");
    FAIL();
  } catch (const ClientError & error) {
    ASSERT_TRUE(error.status);
    EXPECT_EQ(error.kind, ErrorKind::SERVER);
    EXPECT_EQ(error.status->code, Status::BUSY);
  }
  EXPECT_THROW(client_->can_transform("map", "busy"), ClientError);
  EXPECT_THROW(client_->lookup_transform("map", "base", rclcpp::Time(0, 0), -1ns),
      std::invalid_argument);
  Client absent(node_, "absent");
  EXPECT_FALSE(absent.wait_for_server(10ms));
  EXPECT_THROW(absent.get_frames_async().get(), ClientError);
}


TEST_P(Clients, TemporaryClientsReuseNodeOwnedCallbackGroup)
{
  auto group_count = [this]() {
      size_t count = 0;
      node_->get_node_base_interface()->for_each_callback_group(
        [&count](rclcpp::CallbackGroup::SharedPtr) {++count;});
      return count;
    };
  const auto before = group_count();
  for (int index = 0; index < 20; ++index) {
    {
      Client temporary(node_, "absent");
      EXPECT_EQ(group_count(), before);
      EXPECT_THROW(temporary.get_frames_async().get(), ClientError);
    }
    EXPECT_EQ(group_count(), before);
  }
  EXPECT_EQ(client_->get_frames().size(), 2u);
}

TEST_P(Clients, DeadlinesAndClose)
{
  for (int index = 0; index < 3; ++index) {
    auto pending = client_->lookup_transform_async("map", "never");
    ASSERT_EQ(pending.wait_for(2s), std::future_status::ready);
    try {
      pending.get(); FAIL();
    } catch (const ClientError & error) {
      EXPECT_EQ(error.kind, ErrorKind::RESPONSE_TIMEOUT);
    }
  }
  EXPECT_FALSE(client_->get_frames().empty());
  auto pending = client_->lookup_transform_async("map", "never");
  client_->close();
  EXPECT_THROW(pending.get(), ClientError);
  EXPECT_THROW(client_->get_frames_async().get(), ClientError);
  EXPECT_FALSE(client_->is_ready());
  EXPECT_TRUE(node_->get_node_base_interface()->get_context()->is_valid());
}

TEST_P(Clients, InterfacesAndLifecycleNode)
{
  tf_buffer_server_client::NodeInterfaces interfaces{node_->get_node_base_interface(),
    node_->get_node_graph_interface(), node_->get_node_services_interface(),
    node_->get_node_timers_interface()};
  Client direct(interfaces, "buffer");
  ASSERT_TRUE(direct.wait_for_server(3s));
  EXPECT_EQ(direct.get_frames().size(), 2u);
  auto lifecycle = std::make_shared<rclcpp_lifecycle::LifecycleNode>("managed_client", options_);
  Client managed(lifecycle.get(), "buffer");
  executor_->add_node(lifecycle->get_node_base_interface());
  ASSERT_TRUE(managed.wait_for_server(3s));
  EXPECT_EQ(managed.get_frames().size(), 2u);
  managed.close();
  executor_->remove_node(lifecycle->get_node_base_interface());
}

TEST_P(Clients, BlockingFromCallbackHasBoundedWait)
{
  std::promise<bool> finished;
  auto future = finished.get_future();
  timer_ = node_->create_wall_timer(10ms, [this, &finished]() {
        timer_->cancel();
        try {
          auto frames = client_->get_frames();
          finished.set_value(GetParam() && frames.size() == 2);
        } catch (const ClientError & error) {
          finished.set_value(!GetParam() && error.kind == ErrorKind::RESPONSE_TIMEOUT);
        }
    });
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  EXPECT_TRUE(future.get());
}
INSTANTIATE_TEST_SUITE_P(Executors, Clients, ::testing::Values(false, true));
}  // namespace
