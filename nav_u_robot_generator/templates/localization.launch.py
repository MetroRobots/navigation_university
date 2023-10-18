from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(SetParameter(name='use_sim_time', value=True))

    ld.add_action(DeclareLaunchArgument('amcl', default_value='false'))

    ld.add_action(DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution(
            [FindPackageShare('university_world'), 'maps', 'willow-2010-02-18-0.10.yaml'])))

    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml')}],
    ))

    ld.add_action(Node(
        package='metro_nav_demo_utils',
        executable='odom_to_tf',
        output='screen',
    ))

    ld.add_action(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=IfCondition(LaunchConfiguration('amcl')),
        parameters=[PathJoinSubstitution([FindPackageShare('[CONFIG_PACKAGE_NAME]'), 'config', 'amcl.yaml'])],
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        condition=IfCondition(LaunchConfiguration('amcl')),
        parameters=[{'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('amcl')),
        parameters=[{'autostart': True},
                    {'node_names': ['map_server']}]))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', '/map', '--child-frame-id', '/odom'],
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('[CONFIG_PACKAGE_NAME]'), 'config', 'amcl.rviz'])],
        output='screen'
    ))

    ld.add_action(IncludeLaunchDescription(
        [FindPackageShare('[PACKAGE_NAME]'), '/launch/description.launch.py'],
    ))

    return ld
