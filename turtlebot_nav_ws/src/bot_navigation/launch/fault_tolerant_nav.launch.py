from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bot_navigation',
            executable='fault_injector',
            name='fault_injector',
            output='screen',
            parameters=[{'use_sim_time': True, 'fault_mode': 'none'}],
        ),
        Node(
            package='bot_navigation',
            executable='fault_supervisor',
            name='fault_supervisor',
            output='screen',
            parameters=[{'use_sim_time': True, 'opt_profile': 'balanced'}],
        ),
        Node(
            package='bot_navigation',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
