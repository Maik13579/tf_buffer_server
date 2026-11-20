# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Terminal formatting without graph rendering dependencies."""
from collections import defaultdict
import yaml


def tree(frame_yaml, known_frames=()):
    data = yaml.safe_load(frame_yaml) or {}
    if data == []:
        data = {}
    if not isinstance(data, dict):
        raise ValueError('Server returned an invalid frame graph')
    children = defaultdict(set)
    nodes = set(known_frames)
    parents = {}
    for frame, properties in data.items():
        if not isinstance(frame, str) or not isinstance(properties, dict):
            raise ValueError('Server returned an invalid frame graph entry')
        nodes.add(frame)
        parent = properties.get('parent')
        if parent:
            if not isinstance(parent, str):
                raise ValueError('Server returned an invalid parent frame')
            nodes.add(parent)
            parents[frame] = parent
            children[parent].add(frame)
    visited = set()
    lines = []
    roots = sorted(nodes - parents.keys())
    # A cyclic component has no root. Still show it, with an explicit cycle marker.
    for root in roots + sorted(nodes):
        if root in visited:
            continue
        stack = [(root, '', '', frozenset())]
        while stack:
            node, prefix, branch, ancestors = stack.pop()
            label = prefix + branch + node
            if node in ancestors:
                lines.append(label + ' [cycle]')
                continue
            if node in visited:
                continue
            visited.add(node)
            lines.append(label)
            entries = sorted(children[node])
            next_prefix = prefix + ('    ' if branch == '└── ' else '│   ') if branch else ''
            for index in range(len(entries) - 1, -1, -1):
                child_branch = '└── ' if index == len(entries) - 1 else '├── '
                stack.append((entries[index], next_prefix, child_branch, ancestors | {node}))
    return '\n'.join(lines) if lines else '(no known frames)'


def transform_text(value):
    position, rotation = value.transform.translation, value.transform.rotation
    stamp = value.header.stamp
    return (f'{value.header.frame_id} ← {value.child_frame_id} @ {stamp.sec}.{stamp.nanosec:09d}\n'
            f'  translation [m]: [{position.x:.6g}, {position.y:.6g}, {position.z:.6g}]\n'
            f'  quaternion:      [{rotation.x:.6g}, {rotation.y:.6g}, '
            f'{rotation.z:.6g}, {rotation.w:.6g}]')
