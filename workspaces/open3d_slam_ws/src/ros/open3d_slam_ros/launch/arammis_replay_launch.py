# arammis_replay.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('launch_prefix', default_value='', description='gdb -ex run --args'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('launch_tf', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/point_cloud_filter/lidar/point_cloud_filtered'),
        DeclareLaunchArgument('odometry_topic', default_value='no_odometry_topic'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='/state_estimator/pose_in_odom'),
        DeclareLaunchArgument('parameter_filename', default_value='param_arammis_layered.lua'),
        DeclareLaunchArgument('parameter_folder_path', default_value=[ThisLaunchFileDir(), '/../param/']),
        DeclareLaunchArgument('map_saving_folder', default_value='/tmp/'),
        DeclareLaunchArgument('num_accumulated_range_data', default_value='1'),
        DeclareLaunchArgument('is_read_from_rosbag', default_value='true'),
        DeclareLaunchArgument('rosbag_filepath', default_value='/home/tutuna/Videos/rsl_Seam_Tests/query/query_rubble/merged_query_rubble.bag'),
        DeclareLaunchArgument('use_syncronized_poses_to_replay', default_value='false'),
        DeclareLaunchArgument('republish_tf_topic', default_value='true'),
        DeclareLaunchArgument('async_pose_topic', default_value='/state_estimator/pose_in_odom'),
        DeclareLaunchArgument('relative_sleep_duration', default_value='0.0'),
        DeclareLaunchArgument('replay_start_time_as_second', default_value='0.0'),
        DeclareLaunchArgument('replay_end_time_as_second', default_value='2000.0'),
        DeclareLaunchArgument('assumed_external_odometry_tracked_frame', default_value='base'),
        DeclareLaunchArgument('export_imu_data', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    node_params = {
        'cloud_topic': LaunchConfiguration('cloud_topic'),
        'odometry_topic': LaunchConfiguration('odometry_topic'),
        'pose_stamped_topic': LaunchConfiguration('pose_stamped_topic'),
        'pose_stamped_with_covariance_topic': LaunchConfiguration('pose_stamped_with_covariance_topic'),
        'parameter_folder_path': LaunchConfiguration('parameter_folder_path'),
        'assumed_external_odometry_tracked_frame': LaunchConfiguration('assumed_external_odometry_tracked_frame'),
        'async_pose_topic': LaunchConfiguration('async_pose_topic'),
        'export_imu_data': LaunchConfiguration('export_imu_data'),
        'use_syncronized_poses_to_replay': LaunchConfiguration('use_syncronized_poses_to_replay'),
        'republish_tf_topic': LaunchConfiguration('republish_tf_topic'),
        'relative_sleep_duration': LaunchConfiguration('relative_sleep_duration'),
        'replay_start_time_as_second': LaunchConfiguration('replay_start_time_as_second'),
        'replay_end_time_as_second': LaunchConfiguration('replay_end_time_as_second'),
        'parameter_filename': LaunchConfiguration('parameter_filename'),
        'num_accumulated_range_data': LaunchConfiguration('num_accumulated_range_data'),
        'is_read_from_rosbag': LaunchConfiguration('is_read_from_rosbag'),
        'rosbag_filepath': LaunchConfiguration('rosbag_filepath'),
        'map_saving_folder': LaunchConfiguration('map_saving_folder'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    mapping_node = Node(
        package='open3d_slam_ros',
        executable='mapping_node',
        name='open3d_slam',
        output='screen',
        parameters=[node_params],
        prefix=LaunchConfiguration('launch_prefix'),
    )

    # Conditional static TF publisher node
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rslidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base', 'rslidar'],
        condition=IfCondition(LaunchConfiguration('launch_tf')),
        output='both'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/opt/ros/jazzy/share/rviz_common/default.rviz'],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription(
        declared_args + [
            SetLaunchConfiguration('use_sim_time', LaunchConfiguration('use_sim_time')),
            tf_node,
            mapping_node,
            rviz2_node
        ]
    )
