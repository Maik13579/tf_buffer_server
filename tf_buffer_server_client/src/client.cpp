// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "tf_buffer_server_client/client.hpp"
#include "transport.hpp"
#include <limits>
#include <tf_buffer_server_interfaces/srv/get_frames.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform.hpp>
#include <tf_buffer_server_interfaces/srv/lookup_transform_full.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform.hpp>
#include <tf_buffer_server_interfaces/srv/can_transform_full.hpp>
#include <tf2_msgs/srv/frame_graph.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace tf_buffer_server_client
{
namespace srv = tf_buffer_server_interfaces::srv;
namespace
{
void checked(const Status & status)
{
  if (status.code != Status::OK) {throw ClientError(ErrorKind::SERVER, status.message, status);}
}
bool availability(const Status & status, bool value)
{
  if (status.code == Status::LOOKUP_ERROR || status.code == Status::CONNECTIVITY_ERROR ||
    status.code == Status::EXTRAPOLATION_ERROR || status.code == Status::TIMEOUT) {return false;}
  checked(status);
  return value;
}
builtin_interfaces::msg::Duration duration(Duration value)
{
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(value).count();
  if (value < Duration::zero() || seconds > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument("Timeout must be nonnegative and fit a ROS Duration");
  }
  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<int32_t>(seconds);
  result.nanosec = static_cast<uint32_t>((value - std::chrono::seconds(seconds)).count());
  return result;
}
template<typename ServiceT>
typename ServiceT::Request basic(
  const std::string & target, const std::string & source,
  rclcpp::Time time, Duration timeout)
{
  typename ServiceT::Request request;
  request.target_frame = target;
  request.source_frame = source;
  request.time = time;
  request.timeout = duration(timeout);
  return request;
}
template<typename ServiceT>
typename ServiceT::Request full(
  const std::string & target, rclcpp::Time target_time,
  const std::string & source, rclcpp::Time source_time, const std::string & fixed, Duration timeout)
{
  typename ServiceT::Request request;
  request.target_frame = target;
  request.target_time = target_time;
  request.source_frame = source;
  request.source_time = source_time;
  request.fixed_frame = fixed;
  request.timeout = duration(timeout);
  return request;
}
}  // namespace

struct Client::Impl
{
  NodeInterfaces interfaces;
  ClientOptions options;
  std::shared_ptr<detail::Registry> registry = std::make_shared<detail::Registry>();
  rclcpp::Client<srv::GetFrames>::SharedPtr frames;
  rclcpp::Client<tf2_msgs::srv::FrameGraph>::SharedPtr graph;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear;
  rclcpp::Client<srv::LookupTransform>::SharedPtr lookup;
  rclcpp::Client<srv::LookupTransformFull>::SharedPtr lookup_full;
  rclcpp::Client<srv::CanTransform>::SharedPtr can;
  rclcpp::Client<srv::CanTransformFull>::SharedPtr can_full;
  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr make(const std::string & name)
  {
#ifdef TF_BUFFER_CLIENT_LEGACY_QOS
    const auto qos = rclcpp::ServicesQoS().get_rmw_qos_profile();
#else
    const auto qos = rclcpp::ServicesQoS();
#endif
    return rclcpp::create_client<ServiceT>(interfaces.base, interfaces.graph, interfaces.services,
      name, qos, options.callback_group);
  }
};

Client::Client(NodeInterfaces interfaces, std::string server, ClientOptions options)
: impl_(std::make_unique<Impl>())
{
  if (!interfaces.base || !interfaces.graph || !interfaces.services || !interfaces.timers) {
    throw std::invalid_argument("All node interfaces are required");
  }
  if (options.response_margin <= Duration::zero()) {
    throw std::invalid_argument("Response margin must be positive");
  }
  while (!server.empty() && server.back() == '/') {server.pop_back();}
  if (server.empty()) {throw std::invalid_argument("Server name must be nonempty");}
  impl_->interfaces = std::move(interfaces);
  impl_->options = options;
  if (!impl_->options.callback_group) {
    // Humble's executor caches raw guard-condition pointers for callback groups.
    // Borrow the node-owned group so destroying a temporary client cannot invalidate them.
    impl_->options.callback_group = impl_->interfaces.base->get_default_callback_group();
  }
  impl_->frames = impl_->make<srv::GetFrames>(server + "/get_frames");
  impl_->graph = impl_->make<tf2_msgs::srv::FrameGraph>(server + "/get_frame_graph");
  impl_->clear = impl_->make<std_srvs::srv::Trigger>(server + "/clear");
  impl_->lookup = impl_->make<srv::LookupTransform>(server + "/lookup_transform");
  impl_->lookup_full = impl_->make<srv::LookupTransformFull>(server + "/lookup_transform_full");
  impl_->can = impl_->make<srv::CanTransform>(server + "/can_transform");
  impl_->can_full = impl_->make<srv::CanTransformFull>(server + "/can_transform_full");
}
Client::~Client() {close();}
void Client::close()
{
  std::map<uint64_t, std::function<void()>> pending;
  {
    std::lock_guard<std::mutex> guard(impl_->registry->mutex);
    impl_->registry->closed = true;
    pending.swap(impl_->registry->pending);
  }
  for (const auto & entry : pending) {
    entry.second();
  }
}
bool Client::is_ready() const
{
  std::lock_guard<std::mutex> guard(impl_->registry->mutex);
  return !impl_->registry->closed && impl_->frames->service_is_ready() &&
         impl_->graph->service_is_ready() && impl_->clear->service_is_ready() &&
         impl_->lookup->service_is_ready() && impl_->lookup_full->service_is_ready() &&
         impl_->can->service_is_ready() && impl_->can_full->service_is_ready();
}
bool Client::wait_for_server(Duration timeout) const
{
  if (timeout < Duration::zero()) {
    throw std::invalid_argument("Discovery timeout must be nonnegative");
  }
  {
    std::lock_guard<std::mutex> guard(impl_->registry->mutex);
    if (impl_->registry->closed) {return false;}
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  if (!impl_->frames->wait_for_service(std::max(Duration::zero(),
      std::chrono::duration_cast<Duration>(deadline - std::chrono::steady_clock::now()))))
  {
    return false;
  }
  if (!impl_->graph->wait_for_service(std::max(Duration::zero(),
      std::chrono::duration_cast<Duration>(deadline - std::chrono::steady_clock::now()))))
  {
    return false;
  }
  if (!impl_->clear->wait_for_service(std::max(Duration::zero(),
      std::chrono::duration_cast<Duration>(deadline - std::chrono::steady_clock::now()))))
  {
    return false;
  }
  if (!impl_->lookup->wait_for_service(std::max(Duration::zero(),
      std::chrono::duration_cast<Duration>(deadline - std::chrono::steady_clock::now()))))
  {
    return false;
  }
  if (!impl_->lookup_full->wait_for_service(std::max(Duration::zero(),
      std::chrono::duration_cast<Duration>(deadline - std::chrono::steady_clock::now()))))
  {
    return false;
  }
  if (!impl_->can->wait_for_service(std::max(Duration::zero(),
      std::chrono::duration_cast<Duration>(deadline - std::chrono::steady_clock::now()))))
  {
    return false;
  }
  if (!impl_->can_full->wait_for_service(std::max(Duration::zero(),
      std::chrono::duration_cast<Duration>(deadline - std::chrono::steady_clock::now()))))
  {
    return false;
  }
  return is_ready();
}

std::shared_future<Frames> Client::get_frames_async(Callback<Frames> callback)
{
  auto operation = detail::send<Frames, srv::GetFrames>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->frames, srv::GetFrames::Request(), Duration::zero(),
      [](const srv::GetFrames::Response & response) {
        checked(response.status); return response.frames;
                                                                                                      },
      std::move(callback));
  return operation.future;
}

Frames Client::get_frames()
{
  auto operation = detail::send<Frames, srv::GetFrames>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->frames, srv::GetFrames::Request(), Duration::zero(),
      [](const srv::GetFrames::Response & response) {
        checked(response.status); return response.frames;
                                                                                                      });
  return operation.wait();
}

std::shared_future<std::string> Client::get_frame_graph_async(Callback<std::string> callback)
{
  auto operation = detail::send<std::string, tf2_msgs::srv::FrameGraph>(impl_->registry,
      impl_->interfaces, impl_->options,
    impl_->graph, tf2_msgs::srv::FrameGraph::Request(), Duration::zero(),
      [](const tf2_msgs::srv::FrameGraph::Response & response) {
        return response.frame_yaml;
      }, std::move(callback));
  return operation.future;
}

std::string Client::get_frame_graph()
{
  auto operation = detail::send<std::string, tf2_msgs::srv::FrameGraph>(impl_->registry,
      impl_->interfaces, impl_->options,
    impl_->graph, tf2_msgs::srv::FrameGraph::Request(), Duration::zero(),
      [](const tf2_msgs::srv::FrameGraph::Response & response) {return response.frame_yaml;});
  return operation.wait();
}

std::shared_future<void> Client::clear_async(Callback<void> callback)
{
  auto operation = detail::send<void, std_srvs::srv::Trigger>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->clear, std_srvs::srv::Trigger::Request(), Duration::zero(),
      [](const std_srvs::srv::Trigger::Response & response) {
        if (!response.success) {
          throw ClientError(ErrorKind::SERVER, response.message);
        }
                                                                                                                                             },
      std::move(callback));
  return operation.future;
}

void Client::clear()
{
  auto operation = detail::send<void, std_srvs::srv::Trigger>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->clear, std_srvs::srv::Trigger::Request(), Duration::zero(),
      [](const std_srvs::srv::Trigger::Response & response) {
        if (!response.success) {
          throw ClientError(ErrorKind::SERVER, response.message);
        }
                                                                                                                                             });
  return operation.wait();
}

std::shared_future<Transform> Client::lookup_transform_async(
  const std::string & target,
  const std::string & source, rclcpp::Time time, Duration timeout, Callback<Transform> callback)
{
  auto operation = detail::send<Transform, srv::LookupTransform>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->lookup, basic<srv::LookupTransform>(target, source, time, timeout), timeout,
      [](const srv::LookupTransform::Response & response) {
        checked(response.status); return response.transform;
                                                                                                               },
      std::move(callback));
  return operation.future;
}

Transform Client::lookup_transform(
  const std::string & target, const std::string & source,
  rclcpp::Time time, Duration timeout)
{
  auto operation = detail::send<Transform, srv::LookupTransform>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->lookup, basic<srv::LookupTransform>(target, source, time, timeout), timeout,
      [](const srv::LookupTransform::Response & response) {
        checked(response.status); return response.transform;
                                                                                                               });
  return operation.wait();
}

std::shared_future<Transform> Client::lookup_transform_full_async(
  const std::string & target, rclcpp::Time target_time, const std::string & source,
  rclcpp::Time source_time, const std::string & fixed, Duration timeout,
  Callback<Transform> callback)
{
  auto operation = detail::send<Transform, srv::LookupTransformFull>(impl_->registry,
      impl_->interfaces, impl_->options,
    impl_->lookup_full,
      full<srv::LookupTransformFull>(target, target_time, source, source_time, fixed, timeout),
      timeout,
      [](const srv::LookupTransformFull::Response & response) {
        checked(response.status); return response.transform;
                                                                                                                   },
      std::move(callback));
  return operation.future;
}

Transform Client::lookup_transform_full(
  const std::string & target, rclcpp::Time target_time, const std::string & source,
  rclcpp::Time source_time, const std::string & fixed, Duration timeout)
{
  auto operation = detail::send<Transform, srv::LookupTransformFull>(impl_->registry,
      impl_->interfaces, impl_->options,
    impl_->lookup_full,
      full<srv::LookupTransformFull>(target, target_time, source, source_time, fixed, timeout),
      timeout,
      [](const srv::LookupTransformFull::Response & response) {
        checked(response.status); return response.transform;
                                                                                                                   });
  return operation.wait();
}

std::shared_future<bool> Client::can_transform_async(
  const std::string & target,
  const std::string & source, rclcpp::Time time, Duration timeout, Callback<bool> callback)
{
  auto operation = detail::send<bool, srv::CanTransform>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->can, basic<srv::CanTransform>(target, source, time, timeout), timeout,
      [](const srv::CanTransform::Response & response) {
        return availability(response.status, response.can_transform);
                                                                                                                     },
      std::move(callback));
  return operation.future;
}

bool Client::can_transform(
  const std::string & target, const std::string & source,
  rclcpp::Time time, Duration timeout)
{
  auto operation = detail::send<bool, srv::CanTransform>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->can, basic<srv::CanTransform>(target, source, time, timeout), timeout,
      [](const srv::CanTransform::Response & response) {
        return availability(response.status, response.can_transform);
                                                                                                                     });
  return operation.wait();
}

std::shared_future<bool> Client::can_transform_full_async(
  const std::string & target, rclcpp::Time target_time, const std::string & source,
  rclcpp::Time source_time, const std::string & fixed, Duration timeout, Callback<bool> callback)
{
  auto operation = detail::send<bool, srv::CanTransformFull>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->can_full,
      full<srv::CanTransformFull>(target, target_time, source, source_time, fixed, timeout),
      timeout,
      [](const srv::CanTransformFull::Response & response) {
        return availability(response.status, response.can_transform);
                                                                                                                         },
      std::move(callback));
  return operation.future;
}

bool Client::can_transform_full(
  const std::string & target, rclcpp::Time target_time, const std::string & source,
  rclcpp::Time source_time, const std::string & fixed, Duration timeout)
{
  auto operation = detail::send<bool, srv::CanTransformFull>(impl_->registry, impl_->interfaces,
      impl_->options,
    impl_->can_full,
      full<srv::CanTransformFull>(target, target_time, source, source_time, fixed, timeout),
      timeout,
      [](const srv::CanTransformFull::Response & response) {
        return availability(response.status, response.can_transform);
                                                                                                                         });
  return operation.wait();
}
}  // namespace tf_buffer_server_client
