import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('rov_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_path_rov_gazebo = os.path.join(get_package_share_directory('rov_gazebo'))

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = os.path.join(pkg_path_rov_gazebo, 'worlds', 'sand.world')

    # Config time simulation
    config_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )

    # Check if rviz_view parameter is set to true
    rviz_view = LaunchConfiguration('rviz_view', default='true')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rov_rviz.launch.py')
        ),
        condition=IfCondition(rviz_view),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-v 3 -r ' + world_file
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/rov_max/joint/shell_to_left_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/rov_max/joint/shell_to_right_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/rov_max/joint/shell_to_center_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/rov_max/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/model/rov_max/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/model/rov_max/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',],
        parameters=[{'qos_overrides./model/rov_max/joint/shell_to_vert_thrust_left.subscriber.reliability': 'reliable',
                        'qos_overrides./model/rov_max/joint/shell_to_vert_thrust_right.subscriber.reliability': 'reliable',
                        'qos_overrides./model/rov_max/joint/shell_to_left_thrust.subscriber.reliability': 'reliable',
                        'qos_overrides./model/rov_max/joint/shell_to_right_thrust.subscriber.reliability': 'reliable',
                        'qos_overrides./model/rov_max/joint/shell_to_center_thrust.subscriber.reliability': 'reliable',}],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        config_time,
        rviz_launch,
        gz_sim,
        bridge
    ])