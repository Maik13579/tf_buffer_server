# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Exercise deferred services in the real single-threaded component container."""
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf_buffer_server_interfaces.msg import BufferStatus
from tf_buffer_server_interfaces.srv import GetFrames, LookupTransform


@pytest.mark.launch_test
def generate_test_description():
    container = ComposableNodeContainer(
        name='test_tf_buffer_container', namespace='',
        package='rclcpp_components', executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='tf_buffer_server', plugin='tf_buffer_server::' + plugin,
                name=name + ('_threaded' if spin_thread else ''),
                parameters=[{'listener.spin_thread': spin_thread}],
                remappings=[('/tf', '/container_test/tf'),
                            ('/tf_static', '/container_test/tf_static')])
            for plugin, name in [('BufferServer', 'ordinary'),
                                 ('LifecycleBufferServer', 'managed')]
            for spin_thread in (False, True)],
        output='screen')
    return LaunchDescription([container, launch_testing.actions.ReadyToTest()]), {
        'container': container}


class TestSingleThreadedContainer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('container_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def result(self, future, seconds=10):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=seconds)
        self.assertTrue(future.done(), 'Service response timed out')
        return future.result()

    def client(self, service_type, name):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=15), name)
        return client

    def test_components_and_deferred_lookup(self):
        for name in ('managed', 'managed_threaded'):
            transitions = self.client(ChangeState, '/' + name + '/change_state')
            for transition_id in (1, 3):  # configure, activate
                request = ChangeState.Request()
                request.transition.id = transition_id
                self.assertTrue(self.result(transitions.call_async(request)).success)
        names = ('ordinary', 'managed', 'ordinary_threaded', 'managed_threaded')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = self.node.create_publisher(TFMessage, '/container_test/tf_static', qos)
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() < 4 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(publisher.get_subscription_count(), 4)

        lookups = [self.client(LookupTransform, '/' + name + '/lookup_transform')
                   for name in names]
        request = LookupTransform.Request()
        request.target_frame = 'container_map'
        request.source_frame = 'container_base'
        request.timeout.sec = 5
        pending = [client.call_async(request) for client in lookups]
        # Neither wait may occupy the container's only executor thread.
        for name in names:
            frames = self.client(GetFrames, '/' + name + '/get_frames')
            response = self.result(frames.call_async(GetFrames.Request()), seconds=2)
            self.assertEqual(response.status.code, BufferStatus.OK)
        self.assertTrue(all(not future.done() for future in pending))

        transform = TransformStamped()
        transform.header.frame_id = request.target_frame
        transform.child_frame_id = request.source_frame
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = 7.0
        message = TFMessage()
        message.transforms = [transform]
        publisher.publish(message)
        for future in pending:
            response = self.result(future)
            self.assertEqual(response.status.code, BufferStatus.OK)
            self.assertEqual(response.transform.transform.translation.x, 7.0)

        # Deactivation must finish pending requests before removing services.
        missing = LookupTransform.Request()
        missing.target_frame = request.target_frame
        missing.source_frame = 'missing'
        missing.timeout.sec = 5
        future = lookups[3].call_async(missing)
        frames = self.client(GetFrames, '/managed_threaded/get_frames')
        self.result(frames.call_async(GetFrames.Request()))
        deactivate = ChangeState.Request()
        deactivate.transition.id = 4
        self.assertTrue(self.result(transitions.call_async(deactivate)).success)
        self.assertEqual(self.result(future).status.code, BufferStatus.INACTIVE)

        # Unload the ordinary plugin with a pending request, then reload it.
        from composition_interfaces.srv import ListNodes, UnloadNode, LoadNode
        listing = self.client(ListNodes, '/test_tf_buffer_container/_container/list_nodes')
        nodes = self.result(listing.call_async(ListNodes.Request()))
        node_id = nodes.unique_ids[nodes.full_node_names.index('/ordinary_threaded')]
        future = lookups[2].call_async(missing)
        frames = self.client(GetFrames, '/ordinary_threaded/get_frames')
        self.result(frames.call_async(GetFrames.Request()))
        unload = self.client(UnloadNode, '/test_tf_buffer_container/_container/unload_node')
        unload_request = UnloadNode.Request()
        unload_request.unique_id = node_id
        self.assertTrue(self.result(unload.call_async(unload_request)).success)
        self.assertEqual(self.result(future).status.code, BufferStatus.SHUTTING_DOWN)
        loader = self.client(LoadNode, '/test_tf_buffer_container/_container/load_node')
        load = LoadNode.Request()
        load.package_name = 'tf_buffer_server'
        load.plugin_name = 'tf_buffer_server::BufferServer'
        load.node_name = 'reloaded'
        self.assertTrue(self.result(loader.call_async(load)).success)
        reloaded = self.client(GetFrames, '/reloaded/get_frames')
        self.assertEqual(self.result(reloaded.call_async(GetFrames.Request())).status.code,
                         BufferStatus.OK)


@launch_testing.post_shutdown_test()
class TestContainerExit(unittest.TestCase):
    def test_exit(self, proc_info, container):
        launch_testing.asserts.assertExitCodes(proc_info, process=container)
