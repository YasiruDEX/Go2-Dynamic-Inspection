# aramis_robosense_combined_launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get directory containing this file
    launch_dir = os.path.dirname(os.path.realpath(__file__))

    # Declare shared arguments (add others as needed)
    declared_args = [
        DeclareLaunchArgument('cloud_topic', default_value='/rslidar/points'),
        DeclareLaunchArgument('odometry_topic', default_value='/graph_msf/est_odometry_odom_imu'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='empty'),
        DeclareLaunchArgument('launch_prefix', default_value=''),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('distance_cutoff', default_value='0.2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value='/opt/ros/jazzy/share/rviz2/rviz/default_config.rviz'),
    ]

    # Include open3d_launch.py
    open3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'open3d_launch.py')
        ),
        launch_arguments={
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'launch_prefix': LaunchConfiguration('launch_prefix'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription(
        declared_args + [
            open3d_launch,
        ]
    )
