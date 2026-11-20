# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
import argparse
import os
import signal
import subprocess
import threading
import time
import pytest
import yaml
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from composition_interfaces.srv import LoadNode
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf_buffer_server_interfaces.srv import GetFrames
from tf_buffer_server_cli.command import TfBufferCommand
from tf_buffer_server_cli.discovery import select_server, FrameCompleter
from tf_buffer_server_cli.display import tree


def test_tree_and_server_selection():
    assert tree('base: {parent: map}\nlaser: {parent: base}', ['orphan']) == (
        'map\n└── base\n    └── laser\norphan')
    assert '[cycle]' in tree('a: {parent: b}\nb: {parent: a}')
    assert tree('[]') == '(no known frames)'
    with pytest.raises(ValueError):
        tree('42')
    assert select_server(['/robot/buffer']) == '/robot/buffer'
    with pytest.raises(ValueError, match='Multiple'):
        select_server(['/a', '/b'])
    with pytest.raises(ValueError, match='No active'):
        select_server([])
    parser = argparse.ArgumentParser()
    TfBufferCommand().add_arguments(parser, 'ros2 tf_buffer')
    args = parser.parse_args(['lookup', 'map', 'base', '--time', '1234567890.123456789'])
    assert args.time == 1234567890123456789
    args = parser.parse_args(['lookup', 'map', 'base'])
    action = next(a for a in parser._actions if isinstance(a, argparse._SubParsersAction))
    assert isinstance(next(a for a in action.choices['lookup']._actions if a.dest == 'source').completer, FrameCompleter)
    with pytest.raises(SystemExit):
        parser.parse_args(['echo', 'map', 'base', '--rate', '0'])


@pytest.fixture(scope='module')
def live_server(tmp_path_factory):
    directory = tmp_path_factory.mktemp('tf_cli')
    log = (directory / 'container.log').open('w')
    executable = get_package_prefix('rclcpp_components') + '/lib/rclcpp_components/component_container'
    name = 'tf_cli_container_' + str(os.getpid())
    proc = subprocess.Popen([executable, '--ros-args', '-r', '__node:=' + name],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('tf_cli_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()

    def await_result(future):
        deadline = time.monotonic() + 8
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done(), log.name
        return future.result()

    try:
        loader = node.create_client(LoadNode, '/' + name + '/_container/load_node')
        assert loader.wait_for_service(timeout_sec=8)
        request = LoadNode.Request()
        request.package_name = 'tf_buffer_server'
        request.plugin_name = 'tf_buffer_server::BufferServer'
        request.node_name = 'buffer'
        request.node_namespace = '/tf_cli_test_' + str(os.getpid())
        request.remap_rules = ['/tf:=/cli_fixture/tf', '/tf_static:=/cli_fixture/tf_static']
        response = await_result(loader.call_async(request))
        assert response.success, response.error_message
        server = request.node_namespace + '/buffer'
        publisher = node.create_publisher(TFMessage, '/cli_fixture/tf_static',
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        message = TFMessage()
        for parent, child, x in [('map', 'base_link', 1.0), ('base_link', 'laser', 2.0)]:
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transform.transform.translation.x = x
            message.transforms.append(transform)
        publisher.publish(message)
        frames = node.create_client(GetFrames, server + '/get_frames')
        assert frames.wait_for_service(timeout_sec=5)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if 'laser' in await_result(frames.call_async(GetFrames.Request())).frames:
                break
        else:
            pytest.fail('Static TF did not reach server')
        yield server, directory
    finally:
        executor.shutdown()
        thread.join()
        node.destroy_node()
        context.try_shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        log.close()


def cli(server, *arguments):
    return subprocess.run(['ros2', 'tf_buffer', *arguments, '--server', server],
                          capture_output=True, text=True, timeout=10)


def test_live_frames_tree_and_transforms(live_server):
    server, _ = live_server
    result = cli(server, 'frames')
    assert result.returncode == 0, result.stderr
    assert set(result.stdout.split()) == {'map', 'base_link', 'laser'}
    result = cli(server, 'tree')
    assert result.returncode == 0, result.stderr
    assert '└── base_link' in result.stdout and '└── laser' in result.stdout
    result = cli(server, 'lookup', 'map', 'laser', '--format', 'yaml')
    assert result.returncode == 0, result.stderr
    assert yaml.safe_load(result.stdout)['transform']['translation']['x'] == 3.0
    result = cli(server, 'lookup', 'map', 'laser', '--fixed-frame', 'base_link',
                 '--source-time', '10', '--target-time', '12')
    assert result.returncode == 0, result.stderr
    assert 'map ← laser' in result.stdout
    assert cli(server, 'can', 'map', 'laser').returncode == 0
    assert cli(server, 'can', 'map', 'missing', '--timeout', '0').returncode == 1
    servers = subprocess.run(['ros2', 'tf_buffer', 'servers'], capture_output=True, text=True, timeout=10)
    assert server in servers.stdout


def test_real_shell_completion(live_server):
    server, directory = live_server
    line = f'ros2 tf_buffer lookup --server {server} map ba'
    env = dict(os.environ, _ARGCOMPLETE='1', COMP_LINE=line, COMP_POINT=str(len(line)),
               _ARGCOMPLETE_IFS='\n', _ARGCOMPLETE_SUPPRESS_SPACE='1')
    output = directory / 'completion.txt'
    response = subprocess.run(['bash', '-c', 'ros2 8>"$1"', 'completion', str(output)],
                              env=env, capture_output=True, text=True, timeout=5)
    assert response.returncode == 0, response.stderr
    assert 'base_link' in output.read_text().splitlines()


def test_unavailable_server_is_bounded(live_server):
    result = cli('/does_not_exist', 'frames', '--discovery-timeout', '0.1')
    assert result.returncode == 1
    assert 'No active TF buffer server' in result.stderr


def test_echo_shutdown_and_clear(live_server):
    import selectors
    server, _ = live_server
    proc = subprocess.Popen(['ros2', 'tf_buffer', 'echo', 'map', 'laser', '--server', server,
                             '--rate', '2'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, start_new_session=True)
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(proc.stdout, selectors.EVENT_READ)
            assert selector.select(timeout=5), 'echo did not produce a transform'
            assert 'map ← laser' in proc.stdout.readline()
        os.killpg(proc.pid, signal.SIGINT)
        _, stderr = proc.communicate(timeout=5)
        assert proc.returncode == 130, stderr
        assert 'Traceback' not in stderr
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
    response = cli(server, 'clear')
    assert response.returncode == 0, response.stderr
    assert 'static TF preserved' in response.stdout
    assert yaml.safe_load(cli(server, 'clear', '--format', 'yaml').stdout) == {'success': True}
    assert cli(server, 'lookup', 'map', 'laser').returncode == 0
