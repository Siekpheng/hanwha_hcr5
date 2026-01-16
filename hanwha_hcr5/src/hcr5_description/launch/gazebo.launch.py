import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Declare Arguments
    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Start Gazebo if true, otherwise connect to real robot'
    )
    
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    
    # 2. Paths
    pkg_description = get_package_share_directory("hcr5_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    xacro_file = os.path.join(pkg_description, "urdf", "hcr5.xacro")
    
    # Path to RViz config (Make sure this folder/file exists or remove the -d argument)
    rviz_config_path = os.path.join(pkg_description, "rviz", "hcr5.rviz")

    # 3. Dynamic Xacro Processing
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file, ' use_mock_hardware:=', use_mock_hardware]),
        value_type=str
    )

    # 4. Environment for Simulation
    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_description, '..')],
        condition=IfCondition(use_mock_hardware)
    )

    # 5. Gazebo Sim Launch (SIM ONLY)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        condition=IfCondition(use_mock_hardware)
    )

    # 6. The Clock/Camera Bridge (SIM ONLY)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        condition=IfCondition(use_mock_hardware)
    )

    # 7. Robot State Publisher (ALWAYS RUN)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_mock_hardware, 
            "robot_description": robot_description_content
        }],
    )

    # 8. RViz Node (ALWAYS RUN)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # 9. Spawn Robot in Gazebo (SIM ONLY)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "hcr5"],
        condition=IfCondition(use_mock_hardware)
    )

    # 10. Controller Spawners (ALWAYS RUN)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_mock_hardware}]
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hcr5_controllers"],
        parameters=[{"use_sim_time": use_mock_hardware}]
    )

    return LaunchDescription([
        declare_use_mock_hardware,
        set_gz_resource_path,
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        arm_controller
    ])