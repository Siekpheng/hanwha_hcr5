import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Setup Paths
    pkg_description = FindPackageShare('hcr5_description')
    pkg_moveit_config = FindPackageShare('hcr5_moveit_config')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    cube_file = PathJoinSubstitution([pkg_description, 'models', 'purple_cube.sdf'])
    cube2_file = PathJoinSubstitution([pkg_description, 'models', 'blue_cube.sdf'])
    
    xacro_file = PathJoinSubstitution([pkg_description, 'urdf', 'hcr5.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_moveit_config, 'config', 'moveit.rviz'])

    # Add this inside generate_launch_description()
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[PathJoinSubstitution([pkg_description, '..'])]
    )

    # 2. Robot Description (URDF)
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 3. Gazebo Simulation (The Body)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Clock Bridge (The Heartbeat)
    # This bridges Gazebo's Sim Time to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/camera1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                    '/hcr5/vacuum_on@std_msgs/msg/Bool[gz.msgs.Boolean'
                ],
        output='screen'
    )

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )

    # 6. Spawn Robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "hcr5"],
    )

    # 7. MoveGroup (The Brain)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit_config, 'launch', 'move_group.launch.py'])
        ),
    )

    # 8. RViz (The Eyes)
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit_config, 'launch', 'moveit_rviz.launch.py'])
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    # 9. Controller Spawners (The Muscles)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hcr5_controllers"],
    )

    spawn_cube = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", cube_file,
            "-name", "purple_cube",
            "-x", "0.2",   # 40cm in front of robot base
            "-y", "0.3",
            "-z", "0.9"   # 70cm table height + 5cm to drop onto it
        ],
        output='screen'
    )

    spawn_cube2 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", cube2_file,
            "-name", "blue_cube",
            "-x", "0.1",   # 40cm in front of robot base
            "-y", "0.4",
            "-z", "0.9"   # 70cm table height + 5cm to drop onto it
        ],
        output='screen'
    )

    return LaunchDescription([
        # Force EVERYTHING in this file to use Sim Time
        SetParameter(name='use_sim_time', value=True),
        set_gz_resource_path,
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_entity,
        move_group,
        rviz_node,
        joint_state_broadcaster,
        arm_controller,
        spawn_cube,
        spawn_cube2
    ])