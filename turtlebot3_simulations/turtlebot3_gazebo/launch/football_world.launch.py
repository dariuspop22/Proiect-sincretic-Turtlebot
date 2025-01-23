from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    world_path = pkg_share + '/worlds/football_world.world'

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='SDF world file'),

        Node(
            package='gazebo_ros',
            executable='gzserver',
            output='screen',
            arguments=['--verbose', LaunchConfiguration('world')]),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'),
    ])
