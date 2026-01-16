from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    # Package paths
    pkg_path = get_package_share_directory('hcr5_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'hcr5.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'hcr5.rviz')

    # Convert Xacro to URDF
    robot_description = subprocess.check_output(['xacro', xacro_file]).decode()

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI (optional for manual testing)
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 with custom config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Launch description
    ld = LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node
    ])

    return ld
