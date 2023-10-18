from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_path = get_package_share_path('[CONFIG_PACKAGE_NAME]')
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        [FindPackageShare('[PACKAGE_NAME]'), '/launch/gazebo.launch.py'],

    ))

    ld.add_action(DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution(
            [FindPackageShare('university_world'), 'maps', 'willow-2010-02-18-0.10.yaml'])))

    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[str(nav_path / 'config' / 'planner.yaml')],
    ))

    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml')}],
    ))

    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[str(nav_path / 'config' / 'controller.yaml')],
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', '/map', '--child-frame-id', '/odom'],
    ))

    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[str(nav_path / 'config' / 'recoveries.yaml')],
    ))

    bt_path = get_package_share_path('nav2_bt_navigator')
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[str(nav_path / 'config' / 'bt_nav.yaml'), {'default_bt_xml_filename': str(
            bt_path / 'behavior_trees' / 'navigate_w_replanning_and_recovery.xml')}],
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names':
                        ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']}]))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('[CONFIG_PACKAGE_NAME]'), 'config', 'local.rviz'])],
        output='screen'
    ))

    return ld
