// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER__SERVER_HPP_
#define TF_BUFFER_SERVER__SERVER_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_msgs/srv/frame_graph.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf_buffer_server_interfaces/msg/buffer_status.hpp>
#include <tf_buffer_server_interfaces/srv/get_frames.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform_full.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform_full.hpp>
#include "steady_timers.hpp"

namespace tf_buffer_server
{
using Status = tf_buffer_server_interfaces::msg::BufferStatus;
namespace srv = tf_buffer_server_interfaces::srv;

struct Settings
{
  double cache_time;
  double max_timeout;
  size_t max_pending;
  size_t tf_depth;
  size_t static_depth;
  bool spin_thread;
};

/// Normalized request used by all four transform services.
struct Query
{
  std::string target, source, fixed;
  tf2::TimePoint target_time, source_time;
  tf2::Duration timeout;
};
Query query_from(const srv::LookupTransform::Request & request);
Query query_from(const srv::LookupTransformFull::Request & request);
Query query_from(const srv::CanTransform::Request & request);
Query query_from(const srv::CanTransformFull::Request & request);
void set_result(srv::LookupTransform::Response &, const geometry_msgs::msg::TransformStamped &);
void set_result(srv::LookupTransformFull::Response &, const geometry_msgs::msg::TransformStamped &);
void set_result(srv::CanTransform::Response &, const geometry_msgs::msg::TransformStamped &);
void set_result(srv::CanTransformFull::Response &, const geometry_msgs::msg::TransformStamped &);
Status status(uint8_t code, const std::string & message);
Status exception_status(const std::exception & error);

/// Shared implementation. Executor callbacks are serialized by the default callback group.
/// Listener and clock callbacks wake an executor timer; they never modify pending requests.
class Server : public std::enable_shared_from_this<Server>
{
public:
  Server(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers,
    const Settings & settings, rclcpp::Logger logger);
  ~Server();
  void initialize();

  template<typename NodeT>
  void start(NodeT * node)
  {
    handle_clock_reset();
    buffer_->clear();
    listen(node);
    active_ = true;
    add_query_service<srv::LookupTransform>(node, "~/lookup_transform");
    add_query_service<srv::LookupTransformFull>(node, "~/lookup_transform_full");
    add_query_service<srv::CanTransform>(node, "~/can_transform");
    add_query_service<srv::CanTransformFull>(node, "~/can_transform_full");
    services_.push_back(node->template create_service<srv::GetFrames>("~/get_frames",
      [this](srv::GetFrames::Request::SharedPtr, srv::GetFrames::Response::SharedPtr response) {
        handle_clock_reset();
        buffer_->_getFrameStrings(response->frames);
        std::sort(response->frames.begin(), response->frames.end());
        response->frames.erase(std::unique(response->frames.begin(), response->frames.end()),
          response->frames.end());
        response->status = status(Status::OK, "OK");
      }));
    services_.push_back(node->template create_service<tf2_msgs::srv::FrameGraph>(
        "~/get_frame_graph",
        [this](tf2_msgs::srv::FrameGraph::Request::SharedPtr,
        tf2_msgs::srv::FrameGraph::Response::SharedPtr response) {
          handle_clock_reset();
          response->frame_yaml = buffer_->allFramesAsYAML(tf2_ros::fromRclcpp(clock_->now()));
      }));
    services_.push_back(node->template create_service<std_srvs::srv::Trigger>("~/clear",
      [this, node](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_clock_reset();
        // Finish any native TF callback before changing the cache it is reading.
        listener_.reset();
        cancel_all(Status::CLEARED, "Dynamic cache cleared");
        buffer_->clear();
        listen(node);
        response->success = true;
        response->message = "Dynamic cache cleared; static transforms preserved";
      }));
  }
  void stop(uint8_t code = Status::INACTIVE, const std::string & reason = "Server inactive");

private:
  template<typename NodeT>
  void listen(NodeT * node)
  {
    listener_ = std::make_unique<tf2_ros::TransformListener>(
      *buffer_, node, settings_.spin_thread, tf2_ros::DynamicListenerQoS(settings_.tf_depth),
      tf2_ros::StaticListenerQoS(settings_.static_depth));
  }

  using Reply = std::function<void(const Status &, const geometry_msgs::msg::TransformStamped &)>;
  struct Pending
  {
    Query query;
    Reply reply;
    std::chrono::steady_clock::time_point deadline;
    std::vector<tf2_ros::TimerHandle> timers;
    size_t remaining;
    bool done = false;
  };

  template<typename ServiceT, typename NodeT>
  void add_query_service(NodeT * node, const std::string & name)
  {
    auto callback = [this](std::shared_ptr<rclcpp::Service<ServiceT>> service,
      std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<typename ServiceT::Request> request) {
        Reply reply = [this, service, header](const Status & result,
          const geometry_msgs::msg::TransformStamped & transform) {
            typename ServiceT::Response response;
            response.status = result;
            if (result.code == Status::OK) {
              set_result(response, transform);
            }
            try {
              service->send_response(*header, response);
            } catch (const std::exception & error) {
              RCLCPP_WARN(logger_, "Cannot send service response: %s", error.what());
            }
          };
        try {
          submit(query_from(*request), reply);
        } catch (const std::exception & error) {
          reply(exception_status(error), geometry_msgs::msg::TransformStamped());
        }
      };
    services_.push_back(node->template create_service<ServiceT>(name, callback));
  }

  void submit(const Query & query, Reply reply);
  void register_wait(
    const std::shared_ptr<Pending> &, const std::string & target,
    const std::string & source, tf2::TimePoint time);
  void ready(
    const std::shared_ptr<Pending> &, const tf2_ros::TransformStampedFuture &,
    const std::string & target, const std::string & source, tf2::TimePoint time);
  void finish(
    const std::shared_ptr<Pending> &, const Status &,
    const geometry_msgs::msg::TransformStamped & transform = geometry_msgs::msg::TransformStamped());
  void cancel_all(uint8_t code, const std::string & reason);
  void handle_clock_reset();
  void dispatch_events();
  struct Completion
  {
    std::weak_ptr<Pending> request;
    tf2_ros::TransformStampedFuture future;
    std::string target, source;
    tf2::TimePoint time;
  };
  void enqueue(Completion completion);
  geometry_msgs::msg::TransformStamped lookup(const Query &) const;

  Settings settings_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<SteadyTimers> timers_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::list<std::shared_ptr<Pending>> pending_;
  bool active_ = false;
  std::atomic<bool> clock_reset_{false};
  // Only this small mailbox is shared with the optional listener thread.
  std::mutex events_mutex_;
  std::vector<Completion> completions_;
  rclcpp::TimerBase::SharedPtr dispatch_timer_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
};
}  // namespace tf_buffer_server
#endif
