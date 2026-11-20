# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

package_name = 'tf_buffer_server_cli'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=False,
    maintainer='Maik Knof',
    maintainer_email='maik.knof@gmx.de',
    license='Apache-2.0',
    description='Inspect remote TF buffers through ros2 tf_buffer.',
    entry_points={
        'ros2cli.command': [
            'tf_buffer = tf_buffer_server_cli.command:TfBufferCommand',
        ],
    },
)
