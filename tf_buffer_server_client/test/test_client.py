# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
import threading
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf_buffer_server_interfaces.msg import BufferStatus as Status
from tf_buffer_server_interfaces.srv import GetFrames, LookupTransform, LookupTransformFull, CanTransform, CanTransformFull
from tf2_msgs.srv import FrameGraph
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
from tf_buffer_server_client import Client, ClientError


@pytest.fixture
def fixture():
    context = Context()
    rclpy.init(context=context)
    server = Node('server', namespace='/python_tf_client_test', context=context)
    node = Node('client', namespace='/python_tf_client_test', context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)

    def frames(request, response):
        response.frames = ['map', 'base']
        return response

    def lookup(request, response):
        response.status.code = Status.BUSY if request.source_frame == 'busy' else Status.OK
        response.status.message = 'test response'
        response.transform.header.frame_id = request.target_frame
        response.transform.child_frame_id = request.source_frame
        response.transform.header.stamp = request.time
        if request.source_frame == 'slow':
            time.sleep(0.15)
        return response

    def full(request, response):
        response.transform.header.frame_id = request.fixed_frame
        response.transform.transform.translation.x = float(request.target_time.sec - request.source_time.sec)
        return response

    def can(request, response):
        response.status.code = Status.LOOKUP_ERROR if request.source_frame == 'missing' else Status.OK
        response.can_transform = response.status.code == Status.OK
        return response

    def can_full(request, response):
        response.can_transform = request.fixed_frame == 'map'
        return response

    def graph(request, response):
        response.frame_yaml = 'base: {parent: map}'
        return response

    def clear(request, response):
        response.success = True
        return response

    for name, service, callback in [('get_frames', GetFrames, frames), ('lookup_transform', LookupTransform, lookup),
                                    ('lookup_transform_full', LookupTransformFull, full), ('can_transform', CanTransform, can),
                                    ('can_transform_full', CanTransformFull, can_full), ('get_frame_graph', FrameGraph, graph),
                                    ('clear', Trigger, clear)]:
        server.create_service(service, 'buffer/' + name, callback)
    executor.add_node(server)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    client = Client(node, 'buffer', response_margin=0.5)
    assert client.wait_for_server(5)
    try:
        yield client, node
    finally:
        client.close()
        executor.shutdown()
        thread.join()
        node.destroy_node()
        server.destroy_node()
        context.try_shutdown()


def test_values_and_checked_errors(fixture):
    client, node = fixture
    assert client.get_frames() == ['map', 'base']
    assert client.get_frame_graph() == 'base: {parent: map}'
    assert client.clear() is None
    value = client.lookup_transform('map', 'base', Time(sec=10))
    assert value.header.stamp.sec == 10
    assert client.lookup_transform_full('base', Time(sec=12), 'base', Time(sec=10), 'map').transform.translation.x == 2
    assert client.can_transform('map', 'base')
    assert not client.can_transform('map', 'missing')
    assert client.can_transform_full('base', None, 'base', None, 'map')
    with pytest.raises(ClientError) as error:
        client.lookup_transform('map', 'busy')
    assert error.value.status.code == Status.BUSY
    with pytest.raises(ValueError):
        client.lookup_transform_async('map', 'base', timeout=-1)
    absent = Client(node, 'absent')
    assert not absent.wait_for_server(0)
    with pytest.raises(ClientError) as error:
        absent.get_frames()
    assert error.value.kind == 'unavailable'
    absent.close()


def test_async_callbacks_timeout_cleanup_and_close(fixture):
    client, node = fixture
    event = threading.Event()
    future = client.get_frames_async()
    future.add_done_callback(lambda _: event.set())
    assert event.wait(2)
    assert future.result() == ['map', 'base']
    client.response_margin = 0.05  # Deliberately shorter than the fake server's slow reply.
    for _ in range(3):
        with pytest.raises(ClientError) as error:
            client.lookup_transform('map', 'slow')
        assert error.value.kind == 'response_timeout'
        time.sleep(0.16)
        assert not client._pending
        assert all(not c._pending_requests for c in client._clients.values())
    pending = client.lookup_transform_async('map', 'slow')
    pending.cancel()
    assert pending.cancelled()
    assert not client._pending
    closing = client.lookup_transform_async('map', 'slow')
    client.close()
    with pytest.raises(ClientError) as error:
        closing.result()
    assert error.value.kind == 'closed'
    with pytest.raises(ClientError):
        client.get_frames_async().result()
    assert not client.wait_for_server(0)
    assert node.context.ok()
