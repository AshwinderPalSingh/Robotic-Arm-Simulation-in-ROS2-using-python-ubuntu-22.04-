#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gz   = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_this = FindPackageShare('my_robotic_arm').find('my_robotic_arm')

    world_file = os.path.join(pkg_this, 'worlds', 'empty_world.world')
    urdf_file  = os.path.join(pkg_this, 'urdf', 'my_robot_arm_gazebo.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')
    headless     = LaunchConfiguration('headless',      default='False')
    use_sim      = LaunchConfiguration('use_simulator', default='True')
    world_arg    = LaunchConfiguration('world',         default=world_file)

    declare = lambda n, v: DeclareLaunchArgument(n, default_value=v)
    args = [
        declare('headless', 'False'),
        declare('use_sim_time', 'true'),
        declare('use_simulator', 'True'),
        declare('world', world_file)
    ]

    robot_description = Command(['xacro ', urdf_file])

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': use_sim_time}])

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_sim),
        launch_arguments={'world': world_arg}.items())

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_sim, ' and not ', headless])))

    spawn = Node(package='gazebo_ros',
                 executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description', '-entity', 'my_robot_arm'],
                 parameters=[{'use_sim_time': use_sim_time}],
                 output='screen')

    js_broad = Node(package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster',
                               '--controller-manager', '/controller_manager'])

    pos_ctrl = Node(package='controller_manager',
                    executable='spawner',
                    arguments=['position_controller',
                               '--controller-manager', '/controller_manager'])

    ld = LaunchDescription(args)
    for a in (gz_server, gz_client, rsp, spawn, js_broad, pos_ctrl):
        ld.add_action(a)
    return ld

