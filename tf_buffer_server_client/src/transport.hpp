// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#ifndef TF_BUFFER_SERVER_CLIENT__TRANSPORT_HPP_
#define TF_BUFFER_SERVER_CLIENT__TRANSPORT_HPP_
#include <algorithm>
#include <map>
#include <type_traits>
#include <mutex>
#include "tf_buffer_server_client/client.hpp"

namespace tf_buffer_server_client::detail
{
struct Registry
{
  std::mutex mutex;
  bool closed = false;
  uint64_t next_id = 0;
  std::map<uint64_t, std::function<void()>> pending;
};

template<typename T>
struct Operation
{
  std::shared_future<T> future;
  Duration budget;
  std::function<void()> expire;
  T wait()
  {
    if (future.wait_for(budget) != std::future_status::ready) {expire();}
    return future.get();
  }
};

template<typename T>
struct Pending
{
  std::mutex mutex;
  bool done = false;
  uint64_t id = 0;
  std::weak_ptr<Registry> registry;
  std::promise<T> promise;
  std::shared_future<T> future = promise.get_future().share();
  std::function<void()> cleanup;
  rclcpp::TimerBase::SharedPtr timer;
  Callback<T> callback;

  void finish(std::function<T()> value, std::exception_ptr error = {})
  {
    std::unique_lock<std::mutex> lock(mutex);
    if (done) {return;}
    done = true;
    auto release = std::move(cleanup);
    auto deadline = std::move(timer);
    lock.unlock();
    if (deadline) {deadline->cancel();}
    if (release) {release();}
    if (auto owner = registry.lock()) {
      std::lock_guard<std::mutex> guard(owner->mutex);
      owner->pending.erase(id);
    }
    try {
      if (error) {std::rethrow_exception(error);}
      if constexpr (std::is_void_v<T>) {value(); promise.set_value();} else {
        promise.set_value(value());
      }
    } catch (...) {promise.set_exception(std::current_exception());}
    if (callback) {
      try {callback(future);} catch (const std::exception & exception) {
        RCLCPP_ERROR(rclcpp::get_logger("tf_buffer_server_client"),
          "Response callback failed: %s", exception.what());
      }
    }
  }
  void fail(ErrorKind kind, const std::string & message)
  {finish({}, std::make_exception_ptr(ClientError(kind, message)));}
};

template<typename T, typename ServiceT, typename Extract>
Operation<T> send(
  const std::shared_ptr<Registry> & registry, const NodeInterfaces & interfaces,
  const ClientOptions & options, typename rclcpp::Client<ServiceT>::SharedPtr client,
  const typename ServiceT::Request & request, Duration wait, Extract extract,
  Callback<T> callback = {})
{
  auto pending = std::make_shared<Pending<T>>();
  pending->registry = registry;
  pending->callback = std::move(callback);
  if (wait > Duration::max() - options.response_margin) {
    throw std::invalid_argument("Response deadline exceeds timer range");
  }
  const auto budget = wait + options.response_margin;
  Operation<T> operation{pending->future, budget, [pending]() {
      pending->fail(ErrorKind::RESPONSE_TIMEOUT, "Timed out waiting for service response");
    }};
  std::unique_lock<std::mutex> lock(pending->mutex);
  bool closed;
  {
    std::lock_guard<std::mutex> guard(registry->mutex);
    closed = registry->closed;
    if (!closed) {
      pending->id = ++registry->next_id;
      registry->pending[pending->id] = [pending]() {
          pending->fail(ErrorKind::CLOSED, "Client closed");
        };
    }
  }
  if (closed) {
    pending->registry.reset();
    lock.unlock();
    pending->fail(ErrorKind::CLOSED, "Client closed");
    return operation;
  }
  try {
    if (!client->service_is_ready()) {
      throw ClientError(ErrorKind::UNAVAILABLE,
          std::string("Service unavailable: ") + client->get_service_name());
    }
    auto raw = client->async_send_request(std::make_shared<typename ServiceT::Request>(request),
        [pending, extract](typename rclcpp::Client<ServiceT>::SharedFuture response) {
          pending->finish([response, extract]() {return extract(*response.get());});
      });
    pending->cleanup = [client, id = raw.request_id]() {client->remove_pending_request(id);};
    pending->timer = rclcpp::create_wall_timer(budget, operation.expire, options.callback_group,
      interfaces.base.get(), interfaces.timers.get());
  } catch (...) {
    auto error = std::current_exception();
    lock.unlock();
    pending->finish({}, error);
    return operation;
  }
  return operation;
}
}  // namespace tf_buffer_server_client::detail
#endif
