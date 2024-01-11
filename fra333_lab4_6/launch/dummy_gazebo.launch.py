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
    pkg_name = 'advance_control'
    file_subpath = 'description/advance_robot.urdf.xacro'

    package_name = 'advance_control'
    rviz_file_name = 'simple_kinematics.rviz'
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        rviz_file_name
    )
    rviz_Node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("advance_control"),
            "config",
            "myrobot_controllers.yaml",
        ]
    )

    config = os.path.join(
        get_package_share_directory('advance_control'),
        'config',
        'via_point.yaml'
        )

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': False}] # add other parameters here if required
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

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    scheduler=Node(
    package = 'advance_control',
    name = 'scheduler_node',
    executable = 'scheduler.py',
    parameters = [config]
    )

    generator=Node(
    package = 'advance_control',
    executable = 'generator.py'
    )

    kinematics_server=Node(
    package = 'advance_control',
    executable = 'kinematics_server.py'
    )

    tracker=Node(
    package = 'advance_control',
    name = 'scheduler_node',
    executable = 'tracker.py',
    parameters = [config]
    )

    proximity_detector=Node(
    package = 'advance_control',
    name = 'scheduler_node',
    executable = 'proximity_detector.py',
    parameters = [config]
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
        rviz_Node,
        generator,
        kinematics_server,
        tracker,
        proximity_detector,
        scheduler
    ])


