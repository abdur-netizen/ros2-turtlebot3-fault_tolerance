import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bot_loc_dir = get_package_share_directory('bot_localization')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    default_map = os.path.join(bot_loc_dir, 'maps', 'virtual_world.yaml')
    ekf_config = os.path.join(bot_loc_dir, 'config', 'ekf.yaml')
    nav2_params = os.path.join(bot_loc_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map YAML file to load',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )
    nav2_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
                'params_file': nav2_params,
            }.items(),
        )],
    )
    rviz_node = TimerAction(
        period=9.0,
        actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            additional_env={'QT_QPA_PLATFORM': 'xcb', 'LIBGL_ALWAYS_SOFTWARE': '1'},
        )],
    )
    ld = LaunchDescription()
    for action in (
        declare_map_cmd, declare_use_sim_time_cmd, set_tb3_model,
        gazebo_launch, ekf_node, nav2_launch, rviz_node,
    ):
        ld.add_action(action)
    return ld
