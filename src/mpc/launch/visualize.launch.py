#!/usr/bin/env python
"""
Example to launch a visualization node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('mpc')
    # 创建启动描述
    ld = LaunchDescription()
    rviz2 = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'rviz/mpc.rviz')]],
            # output='screen',
            # shell=True,
        )
    ld.add_action(rviz2)
    
    # 创建三个robot的命名空间
    robot_namespaces = ["robot0", "robot1", "robot2"]
    # robot_namespaces = ["robot0"]
    for robot_namespace in robot_namespaces:
        vis_node = Node(
                package='mpc',
                namespace=robot_namespace,
                executable='vis.py',
                name='visualize_node',
                output='screen',
                shell=True,
            )
        ld.add_action(vis_node)
    
    return ld