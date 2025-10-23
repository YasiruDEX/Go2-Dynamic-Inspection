# arammis.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments (all corresponding to your XML)
    # Dynamically get the directory of this launch file
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    param_dir = os.path.join(
        get_package_share_directory('open3d_slam_ros'), 'param'
    )

    # Dynamically set the default map_saving_folder relative to this launch file
    default_map_saving_folder = os.path.normpath(
        os.path.join(launch_dir, '..', 'data/')
    )

    declared_args = [
        DeclareLaunchArgument('launch_prefix', default_value='', description='gdb -ex run --args'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'), #/dlio/odom_node/pointcloud/deskewed #'/pointcloud #/rslidar/points
        DeclareLaunchArgument('odometry_topic', default_value='/odom_dlio'), #/graph_msf/est_odometry_odom_imu /dlio/odom_node/odom
        DeclareLaunchArgument('assumed_external_odometry_tracked_frame', default_value='livox_frame'), #imu_link #base_link
        DeclareLaunchArgument('parameter_filename', default_value='param_summer_school.lua'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='empty'),
        DeclareLaunchArgument('parameter_folder_path',default_value=param_dir),
        DeclareLaunchArgument('map_saving_folder', default_value= default_map_saving_folder),
        DeclareLaunchArgument('num_accumulated_range_data', default_value='1'),
        DeclareLaunchArgument('is_read_from_rosbag', default_value='false'),
        DeclareLaunchArgument('rosbag_filepath', default_value=''),
        DeclareLaunchArgument('use_syncronized_poses_to_replay', default_value='false'),
        DeclareLaunchArgument('republish_tf_topic', default_value='true'),
        DeclareLaunchArgument('async_pose_topic', default_value=''),
        DeclareLaunchArgument('relative_sleep_duration', default_value='0.0'),
        DeclareLaunchArgument('replay_start_time_as_second', default_value='0.0'),
        DeclareLaunchArgument('replay_end_time_as_second', default_value='2000.0'),
        DeclareLaunchArgument('export_imu_data', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    # Node parameters
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

    # Main SLAM node
    mapping_node = Node(
        package='open3d_slam_ros',
        executable='mapping_node',
        name='open3d_slam',
        namespace='open3d',
        output='screen',
        parameters=[node_params],
        prefix=LaunchConfiguration('launch_prefix'),
        respawn=True,
        respawn_delay=0.0,
    )

    launch_dir = os.path.dirname(os.path.realpath(__file__))
    rviz_config_path = os.path.join(
        get_package_share_directory('open3d_slam_ros'), 'rviz', 'ros2.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config_path,
            '--ros-args', '--log-level', 'WARN'
        ],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # Compose launch description
    return LaunchDescription(
        declared_args + [
            SetLaunchConfiguration('use_sim_time', LaunchConfiguration('use_sim_time')),
            mapping_node,
            rviz2_node,
        ]
    )
