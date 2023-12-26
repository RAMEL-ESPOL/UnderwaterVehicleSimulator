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
    pkg_path = get_package_share_directory('auv_max_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_path_auv_max_gazebo = os.path.join(get_package_share_directory('auv_max_gazebo'))

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = os.path.join(pkg_path_auv_max_gazebo, 'worlds', 'sand.world')

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
            os.path.join(pkg_path, 'launch', 'max_rviz.launch.py')
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
        arguments=['/model/auv_max/joint/shell_to_vert_thrust_left/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/auv_max/joint/shell_to_vert_thrust_right/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/auv_max/joint/shell_to_left_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/auv_max/joint/shell_to_right_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/auv_max/joint/shell_to_center_thrust/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/auv_max/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/model/auv_max/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/model/auv_max/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/model/auv_max/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                    '/model/auv_max/sonar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/model/auv_max/sonar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    '/model/auv_max/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',],
        parameters=[{'qos_overrides./model/auv_max/joint/shell_to_vert_thrust_left.subscriber.reliability': 'reliable',
                        'qos_overrides./model/auv_max/joint/shell_to_vert_thrust_right.subscriber.reliability': 'reliable',
                        'qos_overrides./model/auv_max/joint/shell_to_left_thrust.subscriber.reliability': 'reliable',
                        'qos_overrides./model/auv_max/joint/shell_to_right_thrust.subscriber.reliability': 'reliable',
                        'qos_overrides./model/auv_max/joint/shell_to_center_thrust.subscriber.reliability': 'reliable',}],
        output='screen'
    )

    max_node = Node(
        package='auv_max_node',
        executable='auv_main_node_main',
        output='screen',
    )

    # Launch!
    return LaunchDescription([
        config_time,
        rviz_launch,
        gz_sim,
        bridge,
        max_node,
    ])