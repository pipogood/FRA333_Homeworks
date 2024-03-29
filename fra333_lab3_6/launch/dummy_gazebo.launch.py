import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'simple_control'
    file_subpath = 'description/simple_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("simple_control"),
            "config",
            "myrobot_controllers.yaml",
        ]
    )

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robot'],
                    output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_raw}, robot_controllers],

        output="both",
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_velocity_controller", "--controller-manager", "/controller_manager"],
    )


    imu_complementary_filter = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
    )

    algorithm = Node(
        package="simple_control",
        executable="jacob_try.py",
    )

    xicro = Node(
        package="xicro_pkg",
        executable="xicro_node_read_imu_ID_3_arduino.py",
    )


    # Run the node
    return LaunchDescription([

        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        #robot_controller_spawner,
        control_node,
        velocity_controller_spawner,
        imu_complementary_filter,
        algorithm,
    ])


