// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "server.hpp"
#include <algorithm>
#include <utility>

namespace tf_buffer_server
{
Server::Server(
  rclcpp::Clock::SharedPtr clock,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers,
  const Settings & settings, rclcpp::Logger logger)
: settings_(settings), logger_(logger), clock_(std::move(clock)),
  buffer_(std::make_shared<tf2_ros::Buffer>(clock_, tf2::durationFromSec(settings.cache_time))),
  timers_(std::make_shared<SteadyTimers>(base, timers))
{
  buffer_->setCreateTimerInterface(timers_);
  // This timer wakes only for completions or clock jumps; it never polls TF.
  dispatch_timer_ = rclcpp::create_wall_timer(std::chrono::nanoseconds(1),
      [this]() {dispatch_events();}, nullptr, base.get(), timers.get());
  dispatch_timer_->cancel();
}

void Server::initialize()
{
  rcl_jump_threshold_t threshold{};
  threshold.min_backward.nanoseconds = -1;
  threshold.on_clock_change = true;
  // Unregistered before destroying the timer; do not transfer ownership to the clock thread.
  jump_handler_ = clock_->create_jump_callback(nullptr, [this](const rcl_time_jump_t &) {
        clock_reset_.store(true);
        dispatch_timer_->reset();
    }, threshold);
}

Server::~Server()
{
  jump_handler_.reset();
  stop(Status::SHUTTING_DOWN, "Server shutting down");
  dispatch_timer_->cancel();
}

void Server::enqueue(Completion completion)
{
  {
    std::lock_guard<std::mutex> lock(events_mutex_);
    completions_.push_back(std::move(completion));
  }
  dispatch_timer_->reset();
}

void Server::dispatch_events()
{
  dispatch_timer_->cancel();
  handle_clock_reset();
  std::vector<Completion> events;
  {
    std::lock_guard<std::mutex> lock(events_mutex_);
    events.swap(completions_);
  }
  for (const auto & event : events) {
    if (auto request = event.request.lock()) {
      ready(request, event.future, event.target, event.source, event.time);
    }
  }
}

void Server::handle_clock_reset()
{
  if (clock_reset_.exchange(false)) {
    cancel_all(Status::CLOCK_RESET, "ROS clock changed or jumped backwards");
  }
}

void Server::stop(uint8_t code, const std::string & reason)
{
  active_ = false;
  cancel_all(code, reason);
  services_.clear();
  listener_.reset();  // Join the listener before the node releases its Server ownership.
  std::lock_guard<std::mutex> lock(events_mutex_);
  completions_.clear();
}

geometry_msgs::msg::TransformStamped Server::lookup(const Query & query) const
{
  // Explicit BufferCore overloads have no blocking timeout path (also on Humble).
  if (query.fixed.empty()) {
    return buffer_->lookupTransform(query.target, query.source, query.source_time);
  }
  return buffer_->lookupTransform(query.target, query.target_time,
    query.source, query.source_time, query.fixed);
}

void Server::submit(const Query & query, Reply reply)
{
  handle_clock_reset();
  if (!active_) {
    reply(status(Status::INACTIVE, "Server inactive"), geometry_msgs::msg::TransformStamped());
    return;
  }
  if (tf2::durationToSec(query.timeout) > settings_.max_timeout) {
    reply(status(Status::INVALID_ARGUMENT, "Timeout exceeds requests.max_timeout_seconds"),
        geometry_msgs::msg::TransformStamped());
    return;
  }
  try {
    auto transform = lookup(query);
    reply(status(Status::OK, "OK"), transform);
    return;
  } catch (const tf2::TransformException & error) {
    if (query.timeout == tf2::Duration::zero()) {
      reply(exception_status(error), geometry_msgs::msg::TransformStamped());
      return;
    }
  }
  if (pending_.size() >= settings_.max_pending) {
    reply(status(Status::BUSY, "Pending request limit reached"),
        geometry_msgs::msg::TransformStamped());
    return;
  }
  auto request = std::make_shared<Pending>();
  request->query = query;
  request->reply = std::move(reply);
  request->deadline = std::chrono::steady_clock::now() + query.timeout;
  request->remaining = query.fixed.empty() ? 1 : 2;
  pending_.push_back(request);
  try {
    if (query.fixed.empty()) {
      register_wait(request, query.target, query.source, query.source_time);
    } else {
      register_wait(request, query.fixed, query.source, query.source_time);
      if (!request->done) {
        register_wait(request, query.target, query.fixed, query.target_time);
      }
    }
  } catch (const std::exception & error) {
    finish(request, exception_status(error));
  }
}

void Server::register_wait(
  const std::shared_ptr<Pending> & request,
  const std::string & target, const std::string & source, tf2::TimePoint time)
{
  const auto remaining = request->deadline - std::chrono::steady_clock::now();
  if (remaining <= tf2::Duration::zero()) {
    finish(request, status(Status::TIMEOUT, "Timed out waiting for transform"));
    return;
  }
  std::weak_ptr<Server> weak_self = shared_from_this();
  std::weak_ptr<Pending> weak_request = request;
  timers_->take_last_created();
  // TF may complete inline or on its listener thread. Marshal both onto our executor.
  buffer_->waitForTransform(target, source, time, remaining,
    [weak_self, weak_request, target, source,
    time](const tf2_ros::TransformStampedFuture & future) {
      auto self = weak_self.lock();
      if (self) {
        self->enqueue({weak_request, future, target, source, time});
      }
    });
  if (auto timer = timers_->take_last_created()) {
    request->timers.push_back(*timer);
  }
}

void Server::ready(
  const std::shared_ptr<Pending> & request,
  const tf2_ros::TransformStampedFuture & future,
  const std::string & target, const std::string & source, tf2::TimePoint time)
{
  handle_clock_reset();
  if (request->done) {
    return;
  }
  if (std::chrono::steady_clock::now() >= request->deadline) {
    finish(request, status(Status::TIMEOUT, "Timed out waiting for transform"));
    return;
  }
  try {
    try {
      future.get();  // TF invokes this callback only after fulfilling the future.
    } catch (const tf2::LookupException &) {
      // TF's async age check can reject TimePointZero after the first dynamic sample.
      // Recheck this leg without waiting; this also preserves useful TF error details.
      buffer_->lookupTransform(target, source, time);
    }
    if (--request->remaining == 0) {
      auto transform = lookup(request->query);
      finish(request, status(Status::OK, "OK"), transform);
    }
  } catch (const std::exception & error) {
    finish(request, exception_status(error));
  }
}

void Server::finish(
  const std::shared_ptr<Pending> & request, const Status & result,
  const geometry_msgs::msg::TransformStamped & transform)
{
  if (request->done) {
    return;
  }
  request->done = true;
  pending_.remove(request);
  for (auto timer : request->timers) {
    timers_->finish(timer);
  }
  request->reply(result, transform);
}

void Server::cancel_all(uint8_t code, const std::string & reason)
{
  while (!pending_.empty()) {
    // Copy: finish erases this element from pending_.
    auto request = pending_.front();
    finish(request, status(code, reason));
  }
}
}  // namespace tf_buffer_server
