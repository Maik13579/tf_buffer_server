# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Bounded graph/service queries, including short-lived shell completion nodes."""
from contextlib import contextmanager
import os
import time


@contextmanager
def ros_session():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context = Context()
    node = executor = None
    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node(f'_tf_buffer_cli_{os.getpid()}', context=context,
                                 start_parameter_services=False, enable_rosout=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        yield node, executor
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()


def server_names(node):
    service_type = 'tf_buffer_server_interfaces/srv/GetFrames'
    return sorted({name[:-len('/get_frames')] for name, types in node.get_service_names_and_types()
                   if name.endswith('/get_frames') and service_type in types})


def discover(node, executor, deadline, explicit=None):
    explicit = ('/' + explicit.strip('/')) if explicit else None
    while True:
        names = server_names(node)
        if explicit and explicit in names:
            return names
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return names
        executor.spin_once(timeout_sec=min(0.02, remaining))


def select_server(names, explicit=None):
    if explicit:
        name = '/' + explicit.strip('/')
        if name not in names:
            raise ValueError(f'No active TF buffer server at {name}; check its lifecycle state')
        return name
    if not names:
        raise ValueError('No active TF buffer server found')
    if len(names) > 1:
        raise ValueError('Multiple TF buffer servers found; use --server: ' + ', '.join(names))
    return names[0]


def connect(node, executor, server, deadline):
    from tf_buffer_server_client import Client
    client = Client(node, server, response_margin=max(0.001, deadline - time.monotonic()))
    while not client.is_ready():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            client.close()
            raise ValueError(f'TF buffer services unavailable at {server}')
        executor.spin_once(timeout_sec=min(0.02, remaining))
    return client


def result(executor, future, timeout):
    executor.spin_until_future_complete(future, timeout_sec=max(0.0, timeout))
    if not future.done():
        future.cancel()
        raise ValueError('Timed out waiting for TF buffer response')
    return future.result()


class ServerCompleter:
    def __call__(self, prefix, parsed_args, **kwargs):
        try:
            with ros_session() as (node, executor):
                names = discover(node, executor, time.monotonic() + 0.5)
                return [name for name in names if name.startswith(prefix)]
        except (Exception, KeyboardInterrupt):
            return []


class FrameCompleter:
    def __call__(self, prefix, parsed_args, **kwargs):
        try:
            with ros_session() as (node, executor):
                deadline = time.monotonic() + 0.5
                explicit = getattr(parsed_args, 'server', None)
                names = discover(node, executor, deadline, explicit)
                server = select_server(names, explicit)
                # Auto-discovery uses its whole window. Reserve a separate bounded RPC budget.
                if not explicit:
                    deadline = time.monotonic() + 0.3
                with connect(node, executor, server, deadline) as client:
                    frames = result(executor, client.get_frames_async(), deadline - time.monotonic())
                    return sorted(name for name in frames if name.startswith(prefix))
        except (Exception, KeyboardInterrupt):
            return []
