# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Typed service helpers on a caller-supplied ROS node. No implicit spinning."""
import math
import time
from threading import RLock
from builtin_interfaces.msg import Duration, Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf_buffer_server_interfaces.srv import (
    GetFrames, LookupTransform, LookupTransformFull, CanTransform, CanTransformFull)
from tf2_msgs.srv import FrameGraph
from std_srvs.srv import Trigger
from ._request import Pending, ResultFuture
from .errors import ClientError, checked, availability


_SERVICES = {
    'get_frames': GetFrames, 'lookup_transform': LookupTransform,
    'lookup_transform_full': LookupTransformFull, 'can_transform': CanTransform,
    'can_transform_full': CanTransformFull, 'get_frame_graph': FrameGraph, 'clear': Trigger,
}


def seconds(value, *, positive=False):
    value = float(value)
    if not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError('Expected finite positive seconds' if positive else
                         'Expected finite nonnegative seconds')
    return value


def stamp(value):
    if value is None:
        return Time()
    result = value.to_msg() if hasattr(value, 'to_msg') else value
    if not isinstance(result, Time) or result.sec < 0 or result.nanosec >= 10**9:
        raise ValueError('Expected a nonnegative ROS time')
    return result


def duration(value):
    nanoseconds = round(seconds(value) * 10**9)
    sec, nsec = divmod(nanoseconds, 10**9)
    if sec > 2**31 - 1:
        raise ValueError('Timeout exceeds ROS Duration range')
    return Duration(sec=sec, nanosec=nsec)


class Client:
    """Reuse seven ROS clients. The node must outlive this wrapper.

    Async methods return rclpy futures. Blocking counterparts require an executor
    running elsewhere; they never spin or take ownership of the supplied node.
    Timeouts use seconds, times accept rclpy Time or builtin_interfaces Time.
    """

    def __init__(self, node, server='tf_buffer_server', *, response_margin=2.0,
                 callback_group=None):
        if not server or not server.rstrip('/'):
            raise ValueError('Server name must be nonempty')
        self._node = node
        self.response_margin = seconds(response_margin, positive=True)
        self._group = callback_group or MutuallyExclusiveCallbackGroup()
        self._lock = RLock()
        self._closed = False
        self._pending = set()
        self._clients = {}
        try:
            for name, service in _SERVICES.items():
                self._clients[name] = node.create_client(
                    service, server.rstrip('/') + '/' + name, callback_group=self._group)
        except Exception:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def is_ready(self):
        with self._lock:
            return not self._closed and all(c.service_is_ready() for c in self._clients.values())

    def wait_for_server(self, timeout=2.0):
        """Bounded discovery check; safe during setup before spinning the node."""
        with self._lock:
            if self._closed:
                return False
        deadline = time.monotonic() + seconds(timeout)
        for client in self._clients.values():
            if not client.wait_for_service(timeout_sec=max(0.0, deadline - time.monotonic())):
                return False
        return self.is_ready()

    def _forget(self, pending):
        with self._lock:
            self._pending.discard(pending)

    def close(self):
        """Fail pending calls locally, release ROS clients, and leave the node running."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            pending = list(self._pending)
        for request in pending:
            request.finish(error=ClientError('Client closed', kind='closed'))
        for client in self._clients.values():
            self._node.destroy_client(client)

    def _send(self, name, request, decode, wait=0.0):
        budget = seconds(wait) + self.response_margin
        if budget * 10**9 > 2**63 - 1:
            raise ValueError('Response deadline exceeds ROS timer range')
        with self._lock:
            if self._closed:
                result = ResultFuture(budget)
                result.set_exception(ClientError('Client closed', kind='closed'))
                return result
            pending = Pending(self, self._clients[name], budget, decode)
            self._pending.add(pending)
        pending.start(request)
        return pending.future

    @staticmethod
    def _basic(service, target, source, time, timeout):
        return service.Request(target_frame=target, source_frame=source,
                               time=stamp(time), timeout=duration(timeout))

    @staticmethod
    def _full(service, target, target_time, source, source_time, fixed, timeout):
        return service.Request(target_frame=target, target_time=stamp(target_time),
                               source_frame=source, source_time=stamp(source_time),
                               fixed_frame=fixed, timeout=duration(timeout))

    def get_frames_async(self):
        return self._send('get_frames', GetFrames.Request(), lambda r: list(checked(r).frames))

    def get_frames(self):
        return self.get_frames_async().wait()

    def get_frame_graph_async(self):
        """Return the server's diagnostic YAML string."""
        return self._send('get_frame_graph', FrameGraph.Request(), lambda r: r.frame_yaml)

    def get_frame_graph(self):
        return self.get_frame_graph_async().wait()

    def clear_async(self):
        def decode(response):
            if not response.success:
                raise ClientError(response.message)
        return self._send('clear', Trigger.Request(), decode)

    def clear(self):
        return self.clear_async().wait()

    def lookup_transform_async(self, target, source, time=None, timeout=0.0):
        return self._send('lookup_transform', self._basic(
            LookupTransform, target, source, time, timeout), lambda r: checked(r).transform, timeout)

    def lookup_transform(self, target, source, time=None, timeout=0.0):
        return self.lookup_transform_async(target, source, time, timeout).wait()

    def lookup_transform_full_async(self, target, target_time, source, source_time,
                                    fixed, timeout=0.0):
        return self._send('lookup_transform_full', self._full(
            LookupTransformFull, target, target_time, source, source_time, fixed, timeout),
            lambda r: checked(r).transform, timeout)

    def lookup_transform_full(self, target, target_time, source, source_time, fixed, timeout=0.0):
        return self.lookup_transform_full_async(
            target, target_time, source, source_time, fixed, timeout).wait()

    def can_transform_async(self, target, source, time=None, timeout=0.0):
        return self._send('can_transform', self._basic(
            CanTransform, target, source, time, timeout), availability, timeout)

    def can_transform(self, target, source, time=None, timeout=0.0):
        return self.can_transform_async(target, source, time, timeout).wait()

    def can_transform_full_async(self, target, target_time, source, source_time, fixed, timeout=0.0):
        return self._send('can_transform_full', self._full(
            CanTransformFull, target, target_time, source, source_time, fixed, timeout),
            availability, timeout)

    def can_transform_full(self, target, target_time, source, source_time, fixed, timeout=0.0):
        return self.can_transform_full_async(
            target, target_time, source, source_time, fixed, timeout).wait()
