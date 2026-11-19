// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER__STEADY_TIMERS_HPP_
#define TF_BUFFER_SERVER__STEADY_TIMERS_HPP_

#include <map>
#include <mutex>
#include <optional>
#include <tf2_ros/create_timer_ros.hpp>

namespace tf_buffer_server
{
/// TF supplies ROS time to its timer factory. Substitute steady time for service deadlines.
/// TF can remove completed timers on its listener thread.
class SteadyTimers : public tf2_ros::CreateTimerROS
{
public:
  using CreateTimerROS::CreateTimerROS;
  tf2_ros::TimerHandle createTimer(
    rclcpp::Clock::SharedPtr, const tf2::Duration & period,
    tf2_ros::TimerCallbackType callback) override
  {
    const auto handle = CreateTimerROS::createTimer(steady_clock_, period, callback);
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_[handle] = std::move(callback);
    last_created_ = handle;
    return handle;
  }

  void remove(const tf2_ros::TimerHandle & handle) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callbacks_.erase(handle);
    }
    CreateTimerROS::remove(handle);
  }

  std::optional<tf2_ros::TimerHandle> take_last_created()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = last_created_;
    last_created_.reset();
    return result;
  }

  /// Complete TF's timer path now, so TF removes both its request and timer mapping.
  /// Buffer::cancel() alone does not reliably remove that mapping on supported distros.
  /// The owner marks its request completed first, so the resulting callback is ignored.
  void finish(const tf2_ros::TimerHandle handle)
  {
    tf2_ros::TimerCallbackType callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = callbacks_.find(handle);
      if (it == callbacks_.end()) {
        return;
      }
      callback = it->second;
    }
    callback(handle);  // TF removes its own timer; do not hold our mutex here.
  }

private:
  rclcpp::Clock::SharedPtr steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  std::mutex mutex_;
  std::map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> callbacks_;
  std::optional<tf2_ros::TimerHandle> last_created_;
};
}  // namespace tf_buffer_server
#endif
