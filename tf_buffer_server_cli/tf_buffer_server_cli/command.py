# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""ros2 tf_buffer: inspect, query and clear a remote TF buffer."""
import argparse
from decimal import Decimal, InvalidOperation
import math
import sys
import time
from ros2cli.command import CommandExtension
from .discovery import ServerCompleter, FrameCompleter


def seconds(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError('Expected finite nonnegative seconds')
    return number


def positive(value):
    number = seconds(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('Expected positive seconds or rate')
    return number


def timestamp(value):
    if value == 'latest':
        return None
    try:
        number = Decimal(value)
        if not number.is_finite() or number < 0 or number >= 2**31:
            raise ValueError()
        return int(number * 10**9)
    except (ValueError, InvalidOperation):
        raise argparse.ArgumentTypeError('Expected latest or nonnegative ROS seconds') from None


class TfBufferCommand(CommandExtension):
    """Query TF buffers, print frame trees and complete live frame names."""

    def add_arguments(self, parser, cli_name, *, argv=None):
        self.parser = parser
        commands = parser.add_subparsers(dest='operation')
        descriptions = {
            'servers': 'Discover active TF buffer servers',
            'frames': 'List known frame names',
            'tree': 'Print the TF tree, including disconnected roots',
            'lookup': 'Get one transform, optionally at separate source/target times',
            'can': 'Check transform availability (exit 0 when available)',
            'echo': 'Repeatedly print a transform',
            'clear': 'Clear dynamic history and preserve static TF',
        }
        for name, description in descriptions.items():
            command = commands.add_parser(name, help=description, description=description)
            command.add_argument('--discovery-timeout', type=positive, default=1.0,
                                 help='Graph discovery window in seconds (default: 1)')
            if name == 'servers':
                continue
            command.add_argument('--server', help='Server node path; auto-select when unique').completer = ServerCompleter()
            command.add_argument('--response-margin', type=positive, default=2.0,
                                 help='Real-time response allowance beyond TF wait (default: 2 s)')
            command.add_argument('--format', choices=('text', 'yaml'), default='text')
            if name in ('lookup', 'can', 'echo'):
                command.add_argument('target', help='Transform into this frame').completer = FrameCompleter()
                command.add_argument('source', help='Transform from this frame').completer = FrameCompleter()
                command.add_argument('--time', type=timestamp, default=None, help='ROS seconds or latest')
                command.add_argument('--timeout', type=seconds, default=1.0,
                                     help='Maximum server TF wait in seconds (default: 1)')
                if name != 'echo':
                    command.add_argument('--fixed-frame').completer = FrameCompleter()
                    command.add_argument('--target-time', type=timestamp)
                    command.add_argument('--source-time', type=timestamp)
                else:
                    command.add_argument('--rate', type=positive, default=5.0, help='Maximum requests/s')

    def main(self, *, parser, args):
        if args.operation is None:
            self.parser.print_help()
            return 0
        try:
            return run(args)
        except KeyboardInterrupt:
            return 130
        except Exception as error:
            print(f'tf_buffer: {error}', file=sys.stderr)
            return 1


def run(args):
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from tf_buffer_server_client import Client
    from .discovery import ros_session, discover, select_server, result
    from .display import tree, transform_text

    def stamp(nanoseconds):
        return None if nanoseconds is None else Time(nanoseconds=nanoseconds,
                                                     clock_type=ClockType.ROS_TIME)

    with ros_session() as (node, executor):
        names = discover(node, executor, time.monotonic() + args.discovery_timeout,
                         getattr(args, 'server', None))
        if args.operation == 'servers':
            print('\n'.join(names) if names else '(no active TF buffer servers)')
            return 0
        server = select_server(names, args.server)
        with Client(node, server, response_margin=args.response_margin) as client:
            # New DDS clients may still need discovery after graph endpoints are visible.
            deadline = time.monotonic() + args.discovery_timeout
            while not client.is_ready():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ValueError(f'Services unavailable at {server}; check lifecycle state')
                executor.spin_once(timeout_sec=min(remaining, 0.02))

            def get(future, wait=0.0):
                return result(executor, future, wait + args.response_margin + 0.1)

            if args.operation == 'frames':
                frames = get(client.get_frames_async())
                if args.format == 'yaml':
                    import yaml
                    print(yaml.safe_dump(frames, sort_keys=False), end='')
                else:
                    print('\n'.join(frames) if frames else '(no known frames)')
                return 0
            if args.operation == 'tree':
                graph = get(client.get_frame_graph_async())
                print(graph if args.format == 'yaml' else tree(graph, get(client.get_frames_async())))
                return 0
            if args.operation == 'clear':
                get(client.clear_async())
                print('success: true' if args.format == 'yaml' else
                      'Dynamic cache cleared; static TF preserved')
                return 0

            full = getattr(args, 'fixed_frame', None) is not None or any(
                getattr(args, key, None) is not None for key in ('target_time', 'source_time'))
            if full and not args.fixed_frame:
                raise ValueError('Full lookups require --fixed-frame')
            if full and args.time is not None:
                raise ValueError('Use --source-time and --target-time with --fixed-frame')

            def query():
                operation = 'can_transform' if args.operation == 'can' else 'lookup_transform'
                if full:
                    return getattr(client, operation + '_full_async')(
                        args.target, stamp(args.target_time), args.source, stamp(args.source_time),
                        args.fixed_frame, args.timeout)
                return getattr(client, operation + '_async')(
                    args.target, args.source, stamp(args.time), args.timeout)

            while True:
                started = time.monotonic()
                value = get(query(), args.timeout)
                if args.operation == 'can':
                    print('true' if value else 'false')
                    return 0 if value else 1
                if args.format == 'yaml':
                    from rosidl_runtime_py import message_to_yaml
                    if args.operation == 'echo':
                        print('---')
                    print(message_to_yaml(value), end='', flush=True)
                else:
                    print(transform_text(value), flush=True)
                if args.operation != 'echo':
                    return 0
                time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - started)))
