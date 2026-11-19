# TF buffer server

C++ components exposing a shared TF buffer through deferred ROS 2 services.
Supports Humble, Jazzy, and Lyrical. Includes `tf_buffer_server_interfaces` and
`tf_buffer_server`; no action server is started.

```bash
ros2 launch tf_buffer_server server.launch.py
# Or the lifecycle component (starts unconfigured):
ros2 launch tf_buffer_server lifecycle_server.launch.py
ros2 lifecycle set /tf_buffer_server configure
ros2 lifecycle set /tf_buffer_server activate
```

Load `tf_buffer_server::BufferServer` or `tf_buffer_server::LifecycleBufferServer`
into an existing component container. Both single-threaded and multithreaded
executors are supported. Remap `/tf` and `/tf_static` for isolated TF trees.

## Services

All names are private to the node (`/tf_buffer_server/...` by default).

| Name | Type in `tf_buffer_server_interfaces/srv` | Result |
| --- | --- | --- |
| `get_frames` | `GetFrames` | Sorted names known to the buffer |
| `lookup_transform` | `LookupTransform` | Transform from source into target |
| `lookup_transform_full` | `LookupTransformFull` | Two timestamps through a fixed frame |
| `can_transform` | `CanTransform` | Availability, with optional waiting |
| `can_transform_full` | `CanTransformFull` | Availability for a full lookup |
| `get_frame_graph` | `tf2_msgs/srv/FrameGraph` | TF diagnostic YAML |
| `clear` | `std_srvs/srv/Trigger` | Clear dynamic history, preserve static TF |

Check `status.code == BufferStatus.OK` before using custom-service results.
Failures include a status message; transforms remain default-initialized and
availability is false. Frame names can remain known after clearing history.

Times use ROS time; zero selects latest. Timeouts measure elapsed real time,
including when simulation is paused; zero checks immediately. Requests require
nonnegative seconds and nanoseconds below 1e9. Deferred waits use TF callbacks,
without polling. Set `listener.spin_thread: true` to give TF its own listener
thread (default: false). Service completion stays on the node executor. A client timeout does not cancel
the server's wait; it expires at the requested deadline.

```bash
ros2 service call /tf_buffer_server/lookup_transform \
  tf_buffer_server_interfaces/srv/LookupTransform \
  '{target_frame: map, source_frame: base_link, timeout: {sec: 1}}'
ros2 service call /tf_buffer_server/lookup_transform_full \
  tf_buffer_server_interfaces/srv/LookupTransformFull \
  '{target_frame: base_link, target_time: {sec: 12}, source_frame: laser, source_time: {sec: 11}, fixed_frame: odom, timeout: {sec: 1}}'
```

## Parameters and lifecycle

| Parameter | Default |
| --- | ---: |
| `buffer.cache_time_seconds` | 10.0 |
| `requests.max_timeout_seconds` | 5.0 |
| `requests.max_pending` | 256 |
| `subscriptions.tf_depth` | 100 |
| `subscriptions.tf_static_depth` | 100 |
| `listener.spin_thread` | false |

These parameters are fixed at node creation. Cache time and counts must be
positive; maximum timeout can be zero. Durations must be finite and at most
1e9 seconds. Oversized request timeouts are rejected. `use_sim_time` is supported.
Use `params_file:=/path/to/server.yaml` with either launch file.

The lifecycle component receives TF and advertises application services only
while active. Deactivation fails pending requests before removing endpoints.
Reactivation clears dynamic history and preserves static TF. Cleanup releases
all resources. Clear, shutdown, and backward ROS clock jumps/time-source changes
also finish pending requests with an explicit status.
