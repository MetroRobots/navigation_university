import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

EMPTY_URDF = """<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_footprint"/>
</robot>
"""


def generate_launch_description():
    ld = launch.LaunchDescription()
    ld.add_action(Node(
        package='nav_u_robot_generator',
        executable='nav_u_robot_generator',
    ))
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': EMPTY_URDF}],
    ))
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    ))
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [FindPackageShare('nav_u_robot_generator'), '/config/gen.rviz']],
    ))
    return ld
