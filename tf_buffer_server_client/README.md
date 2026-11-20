# C++ and Python clients

`tf_buffer_server_client` wraps all seven buffer services on a node you supply.
The server prefix defaults to `tf_buffer_server`, relative to that node's namespace;
absolute prefixes such as `/robot/tf_buffer_server` work too.

Methods: `get_frames`, `lookup_transform`, `lookup_transform_full`, `can_transform`,
`can_transform_full`, `get_frame_graph`, and `clear`. Every method also has an
`_async` variant. Results are frame lists, `TransformStamped`, booleans, diagnostic
YAML, or no value for `clear`.

## C++

Link `tf_buffer_server_client::tf_buffer_server_client` after
`find_package(tf_buffer_server_client REQUIRED)`.

```cpp
#include <tf_buffer_server_client/client.hpp>
using namespace std::chrono_literals;

// Declare a member on your Node or LifecycleNode:
std::unique_ptr<tf_buffer_server_client::Client> tf_;

// In its constructor body:
tf_ = std::make_unique<tf_buffer_server_client::Client>(this, "/tf_buffer_server");

// Safe in a single-threaded callback: return immediately and handle the ready result.
tf_->lookup_transform_async("odom", "base_link", rclcpp::Time(0, 0, RCL_ROS_TIME), 1s,
  [](std::shared_future<geometry_msgs::msg::TransformStamped> future) {
    try {
      const auto transform = future.get();
      // Use transform here.
    } catch (const tf_buffer_server_client::ClientError & error) {
      // error.kind distinguishes transport failures; error.status contains server errors.
    }
  });
```

Construct with a raw node pointer, shared pointer, or `NodeInterfaces` containing
base, graph, services, and timers interfaces. `ClientOptions` accepts an optional
callback group and `response_margin` (default 2 seconds).

With an executor spinning elsewhere, blocking calls are convenient:

```cpp
if (tf_->wait_for_server(2s)) {
  auto frames = tf_->get_frames();
  auto transform = tf_->lookup_transform("odom", "base_link");
  auto historical = tf_->lookup_transform_full(
    "base_link", rclcpp::Time(12, 0, RCL_ROS_TIME),
    "base_link", rclcpp::Time(10, 0, RCL_ROS_TIME), "odom", 1s);
}
```

## Python

```python
from tf_buffer_server_client import Client, ClientError

# Store this wrapper alongside your existing node.
tf = Client(node, '/tf_buffer_server')
future = tf.lookup_transform_async('odom', 'base_link', timeout=1.0)

def completed(future):
    try:
        transform = future.result()
    except ClientError as error:
        print(error.kind, error.status, str(error))

future.add_done_callback(completed)
```

Async results are `rclpy` futures and support local `cancel()`. Times accept
`rclpy.time.Time`, `builtin_interfaces.msg.Time`, or `None` for latest. Timeouts
are seconds. With an executor running elsewhere, call `tf.get_frames()` or
`tf.lookup_transform('odom', 'base_link')` directly. Use `tf.close()` or
`with Client(node) as tf:` to release clients and complete pending requests.

## Execution and errors

The library never spins, creates a node, or starts an executor thread. Your node
must outlive the client. Async replies and deadline callbacks need its executor.
Blocking calls have a real-time upper bound but can still starve a single-threaded
executor when called from a callback; use async methods there. A separate default
client callback group permits blocking use with a multithreaded executor. C++
callbacks can also run inline for immediate failures or `close()`; keep them short.

The local response deadline is the requested server wait plus `response_margin`.
Expired requests are removed locally; this does not cancel the server's work.
`ClientError` carries the original server status when available. `can_transform`
returns false for lookup, connectivity, extrapolation, or server wait timeout;
invalid requests, busy servers, cache resets and transport failures raise errors.
Zero time means latest; zero server timeout checks immediately. Client shutdown
leaves the supplied node running.
