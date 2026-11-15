# @file            launch/bringup.launch.py
# @description
# @author          rsazid99 <rsazid99@gmail.com>
# @createTime      2026-08-08 03:27:33
# @lastModified    2026-08-08 03:27:33
# Copyright (c) 2026 Sazid Rahman Simanto
# SPDX-License-Identifier: MIT
"""
Brings up the 4WD base: motor control, BNO085 IMU, fused odometry, and
optionally the FlySky radio bridge.

Manual driving from the radio (the default):

    ros2 launch robot_base_controller bringup.launch.py

Autonomous: leaves cmd_vel free for a planner, no radio node at all.

    ros2 launch robot_base_controller bringup.launch.py flysky:=false

Radio alongside autonomy: the bridge publishes somewhere else, so a twist mux
can arbitrate between the two instead of both fighting over cmd_vel.

    ros2 launch robot_base_controller bringup.launch.py \\
        flysky_cmd_vel_topic:=cmd_vel_manual

Any individual part can also be switched off with base:=false, imu:=false,
fusion:=false.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_base = LaunchConfiguration('base')
    use_imu = LaunchConfiguration('imu')
    use_fusion = LaunchConfiguration('fusion')
    use_flysky = LaunchConfiguration('flysky')
    use_imu_tf = LaunchConfiguration('imu_static_tf')

    default_params = PathJoinSubstitution([
        FindPackageShare('robot_base_controller'),
        'config',
        'robot_base_controller.yaml',
    ])

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with parameters for all robot_base_controller nodes.',
        ),
        DeclareLaunchArgument(
            'base', default_value='true',
            description='Run the DDSM115 base controller.',
        ),
        DeclareLaunchArgument(
            'imu', default_value='true',
            description='Run the BNO085 IMU driver.',
        ),
        DeclareLaunchArgument(
            'fusion', default_value='true',
            description='Run the wheel + IMU odometry fusion node.',
        ),
        DeclareLaunchArgument(
            'flysky', default_value='true',
            description=(
                'Run the FlySky i-BUS radio bridge. Set false to run '
                'autonomously and leave cmd_vel to a planner.'
            ),
        ),
        DeclareLaunchArgument(
            'flysky_cmd_vel_topic', default_value='cmd_vel',
            description=(
                'Topic the radio bridge drives. Point it elsewhere '
                '(e.g. cmd_vel_manual) to arbitrate with a planner via a mux.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_static_tf', default_value='true',
            description=(
                'Publish a static base_link -> imu_link transform. '
                'Disable when a robot_state_publisher / URDF already provides it.'
            ),
        ),
    ] + [
        DeclareLaunchArgument(
            name, default_value='0.0',
            description=f'IMU pose relative to base_link: {name[4:]} '
                        f'({"metres" if name[4] in "xyz" else "radians"}).',
        )
        for name in ('imu_x', 'imu_y', 'imu_z', 'imu_roll', 'imu_pitch', 'imu_yaw')
    ]

    nodes = [
        Node(
            package='robot_base_controller',
            executable='base_controller_node',
            name='base_controller',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_base),
        ),
        Node(
            package='robot_base_controller',
            executable='bno085_imu_node',
            name='bno085_imu',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_imu),
        ),
        Node(
            package='robot_base_controller',
            executable='odom_fusion_node',
            name='odom_fusion',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_fusion),
        ),
        Node(
            package='robot_base_controller',
            executable='flysky_ibus_node',
            name='flysky_ibus_bridge',
            output='screen',
            parameters=[
                params_file,
                {'cmd_vel_topic': LaunchConfiguration('flysky_cmd_vel_topic')},
            ],
            condition=IfCondition(use_flysky),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            output='log',
            arguments=[
                '--x', LaunchConfiguration('imu_x'),
                '--y', LaunchConfiguration('imu_y'),
                '--z', LaunchConfiguration('imu_z'),
                '--roll', LaunchConfiguration('imu_roll'),
                '--pitch', LaunchConfiguration('imu_pitch'),
                '--yaw', LaunchConfiguration('imu_yaw'),
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
            condition=IfCondition(use_imu_tf),
        ),
    ]

    return LaunchDescription(args + nodes)
