#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import  LaunchConfiguration


def generate_launch_description():
    
    laser_relay = Node(
        package='laser_relay',
        executable='laser_relay',
        name='laser_relay',
        output='screen',
        emulate_tty=True,
        parameters=[
                {"topic_in": LaunchConfiguration("topic_in1")},
                {"topic_out": LaunchConfiguration("topic_out")}
        ])

    laser_relay2 = Node(
        package='laser_relay',
        executable='laser_relay',
        name='laser_relay2',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"topic_in": LaunchConfiguration("topic_in2")},
            {"topic_out": LaunchConfiguration("topic_out")}
        ])

    return LaunchDescription([
        laser_relay,
        laser_relay2,
    ])
