# ros2 tf_buffer

Inspect an active TF buffer server from the terminal. If exactly one server is
found it is selected automatically; otherwise use `--server /robot/tf_buffer_server`.
Inactive lifecycle servers do not advertise these services.

```bash
ros2 tf_buffer servers
ros2 tf_buffer frames
ros2 tf_buffer tree
ros2 tf_buffer tree --format yaml
ros2 tf_buffer lookup odom base_link
ros2 tf_buffer lookup odom laser --format yaml
ros2 tf_buffer can odom base_link --timeout 0
ros2 tf_buffer echo odom base_link --rate 5
ros2 tf_buffer clear
```

`lookup TARGET SOURCE` transforms from SOURCE into TARGET. Translation is in
meters and rotations are quaternions. `--time` accepts ROS seconds or `latest`.
For separate timestamps, provide a fixed frame:

```bash
ros2 tf_buffer lookup base_link laser \
  --fixed-frame odom --source-time 11.5 --target-time 12.0 --timeout 1
ros2 tf_buffer lookup --server /robot/tf_buffer_server odom base_link
```

`--timeout` controls the server's TF wait (default 1 second). The local response
allowance adds `--response-margin` (default 2 seconds), measured in elapsed real
time. `--discovery-timeout` controls graph discovery for listing or automatic server
selection (default 1 second). An explicit `--server` skips graph enumeration and
connects directly. `--service-timeout` bounds service readiness waiting (default
5 seconds), separately from the response deadline.
`echo` sends one request at a time; Ctrl-C stops it. `can` exits with 0 when
available and 1 otherwise. Command failures print a short error on stderr.

## Shell completion

With normal ROS 2 Bash/zsh completion enabled, Tab completes commands, server
names, target/source frames, and `--fixed-frame`:

```text
ros2 tf_buffer lookup odom ba<Tab>
ros2 tf_buffer lookup --server /robot/tf_buffer_server odom ba<Tab>
```

The shell starts a temporary ROS node during completion. Server names come from
the ROS graph; frame names come from `get_frames`. Discovery and RPC waits are
bounded and unavailable/ambiguous servers produce no frame suggestions. Completion
only reads frame names; lookup, echo and clear execute after pressing Enter.

The terminal tree shows disconnected roots and marks cycles. Known frame names
may remain listed after dynamic samples are cleared; use `can` or `lookup` to
check availability. YAML output is suitable for scripts. No Graphviz is required.
