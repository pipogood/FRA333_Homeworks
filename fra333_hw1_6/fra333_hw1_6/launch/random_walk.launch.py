#!usr/bin/python3
from statistics import variance
from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    # create a place holder for launch description
    launch_description = LaunchDescription()

    ### Example for adding launch argument ###


    # mean = LaunchConfiguration('mean')
    # mean_launch_arg = DeclareLaunchArgument('mean',default_value='0.0')
    # launch_description.add_action(mean_launch_arg)

    rate = LaunchConfiguration('rate')
    rate_launch_arg = DeclareLaunchArgument('rate',default_value='5.0')
    launch_description.add_action(rate_launch_arg)

    # variance = LaunchConfiguration('variance')
    # variance_launch_arg = DeclareLaunchArgument('variance',default_value='0.0')
    # launch_description.add_action(variance_launch_arg)
    #variance = '1.0'
    
    ### Example for adding a node ###
    Node1 = Node(
        package='fra333_hw1_6',
        executable='noise_generator.py',
        namespace= 'linear_noise_generator',
        name='noise_genarator',
        arguments=['1.0','0.1',rate],
        remappings=[
            ('/noise','/linear/noise'),
            ('/set_noise','/linear/set_noise'),
        ]
    )
    launch_description.add_action(Node1)

    Node2 = Node(
        package='fra333_hw1_6',
        executable='noise_generator.py',
        namespace= 'angular_noise_generator',
        name='noise_genarator',
        arguments=['0.0','3.0',rate],
        remappings=[
            ('/noise','/angular/noise'),
            ('/set_noise','/angular/set_noise'),
        ]
    )
    launch_description.add_action(Node2)
    velo = Node(
        package='fra333_hw1_6',
        executable='velocity_mux.py',
        arguments=[rate]
    )

    launch_description.add_action(velo)

    ### Example for execute a shell command in python script ###
    pub_turtle = ExecuteProcess(
        cmd = [[f'ros2 run turtlesim turtlesim_node']],
        shell=True
    )
    launch_description.add_action(pub_turtle)

    pub_linear = ExecuteProcess(
        cmd=[['ros2 service call /linear/set_noise lab1_interfaces/srv/SetNoise "{mean: {data: 1.0}, variance: {data: 0.1}}"',
        ]],
        shell=True
    )
    launch_description.add_action(pub_linear)

    pub_angular = ExecuteProcess(
        cmd=[['ros2 service call /angular/set_noise lab1_interfaces/srv/SetNoise "{mean: {data: 0.0}, variance: {data: 3.0}}"',
        ]],
        shell=True
    )
    launch_description.add_action(pub_angular)

    # vx = 1.0
    # wz = 1.0
    # pub_cmd_vel = ExecuteProcess(
    #     cmd = [[f'ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: {vx}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}"']],
    #     shell=True
    # )
    # launch_description.add_action(pub_cmd_vel)

    # spawn_turtle2 = ExecuteProcess(
    # cmd = [['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],
    # shell=True
    # )
    # launch_description.add_action(spawn_turtle2)

    
    return launch_description

    