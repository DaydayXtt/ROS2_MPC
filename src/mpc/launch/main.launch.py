#!/usr/bin/env python
"""
二阶积分器 MPC
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
import yaml

def read_yaml_config(file_path):
    with open(file_path, 'r') as file:
        try:
            config = yaml.safe_load(file)
            return config
        except yaml.YAMLError as exc:
            print(f"Error reading YAML file: {exc}")
            return None

def generate_launch_description():
    ld = LaunchDescription()
    # 获取URDF文件路径
    pkg_path = get_package_share_directory('odoms')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # 使用命名空间参数
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command
    
    # 创建三个robot的命名空间
    robot_namespaces = ["robot0", "robot1", "robot2"]
    # robot_namespaces = ["robot0"]
    for robot_namespace in robot_namespaces:
        # urdf前缀
        robot_description = ParameterValue(
            Command(['xacro ', urdf_file, ' robot_namespace:=', robot_namespace]),
            value_type=str
        )
        
        # 1. odom积分器
        # 启动robot_state_publisher
        odom_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher', 
            namespace=robot_namespace,
            parameters=[{'robot_description': robot_description
                        }],
            output='screen',
        )
        odom_int = Node(
            package='odoms',
            executable='odom_integrator',
            namespace=robot_namespace,
            output='screen'
        )
        odom_int_action = TimerAction(
            period=0.0,
            actions=[odom_state_publisher, odom_int]
        )
        ld.add_action(odom_int_action)
        
        # 2. 规划节点 - ros2 run mpc main
        planner = Node(
            package='mpc',
            executable='main_node',
            name='mpc_node',
            namespace=robot_namespace,
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug']
        )
        planner_action = TimerAction(
            period=1.0,
            actions=[planner]
        )
        ld.add_action(planner_action)

    return ld