from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'))

    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('[CONFIG_PACKAGE_NAME]'), 'config', 'mapping.rviz'])
    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    ))

    slam_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'offline_launch.py'])
    )
    ld.add_action(slam_launch)

    return ld
