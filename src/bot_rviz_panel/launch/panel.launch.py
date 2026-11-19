"""Open the local scenario editor with a working static map."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = PathJoinSubstitution([
        FindPackageShare('bot_gazebo'), 'maps',
        PythonExpression(["'", LaunchConfiguration('world'), ".yaml'"])])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse', choices=['warehouse', 'house']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': map_file, 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='map_lifecycle', parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             arguments=['-d', PathJoinSubstitution([FindPackageShare('bot_rviz_panel'), 'launch', 'rviz2.rviz'])]),
    ])
