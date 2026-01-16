import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths
    pkg_description = get_package_share_directory("hcr5_description")
    xacro_file = os.path.join(pkg_description, "urdf", "hcr5.xacro")
    rviz_config_path = os.path.join(pkg_description, "rviz", "hcr5.rviz")

    # 2. Process Xacro
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # 3. Robot State Publisher (Static TF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False  # Set to False since we aren't using Gazebo clock
        }],
    )

    # 4. Joint State Publisher GUI (The Sliders)
    # This node creates the window with sliders for each joint
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    # 5. RViz
    rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      arguments=["-d", rviz_config_path],
      parameters=[{"use_sim_time": False}]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])