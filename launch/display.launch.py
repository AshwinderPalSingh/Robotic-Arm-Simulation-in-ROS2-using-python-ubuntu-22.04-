#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg   = FindPackageShare('my_robotic_arm').find('my_robotic_arm')
    urdf  = Command(['xacro ', os.path.join(pkg, 'urdf', 'my_robot_arm.urdf.xacro')])
    rviz2 = os.path.join(pkg, 'rviz', 'robot_arm.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': urdf,
                          'use_sim_time': use_sim_time}]),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             parameters=[{'use_sim_time': use_sim_time}]),

        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', rviz2],
             parameters=[{'use_sim_time': use_sim_time}])
    ])
