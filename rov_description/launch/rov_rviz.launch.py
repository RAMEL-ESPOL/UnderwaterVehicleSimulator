import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # rov_description_dir = get_package_share_directory('rov_description')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('rov_description'))
    xacro_file = os.path.join(pkg_path,'urdf','rov_max.urdf.xacro')
    rviz_file = os.path.join(pkg_path,'rviz','max_view.rviz')
    world_file = os.path.join(pkg_path, 'worlds', 'sand.world')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Config time simulation
    config_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    # Create a joint_state_publisher node
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rov_ramel',
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-v 3 -r ~/ros2_ws/src/rov_robot/rov_description/worlds/sand.world'
        }.items(),
    )

    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                        '/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                        '/model/rov_max/joint/shell_to_left_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                        '/model/rov_max/joint/shell_to_right_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                        '/model/rov_max/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',],
            parameters=[{'qos_overrides./model/rov_max/joint/shell_to_vert_thrust_left.subscriber.reliability': 'reliable',
                            'qos_overrides./model/rov_max/joint/shell_to_vert_thrust_right.subscriber.reliability': 'reliable',
                            'qos_overrides./model/rov_max/joint/shell_to_left_thrust.subscriber.reliability': 'reliable',
                            'qos_overrides./model/rov_max/joint/shell_to_right_thrust.subscriber.reliability': 'reliable'}],
            output='screen'
    )

    # Launch!
    return LaunchDescription([
        config_time,
        node_robot_state_publisher,
        node_joint_state_publisher,
        rviz,
        spawn,
        gz_sim,
        bridge
    ])