from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import FrontendLaunchDescriptionSource  # Add this import


def generate_launch_description():
    """
    Launch the complete pipeline with sequential delays:
    1. DLIO (starts immediately)
    2. Open3D SLAM (after 10 seconds)
    3. Vehicle Simulator (after 15 seconds - 10 + 5)
    4. Far Planner (after 20 seconds - 15 + 5)
    """
    
    # Find package paths
    vehicle_simulator_pkg = FindPackageShare('vehicle_simulator')
    far_planner_pkg = FindPackageShare('far_planner')
    
    # 1. Launch DLIO immediately
    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('direct_lidar_inertial_odometry'), '/launch/dlio.launch.py'
        ]),
        launch_arguments={
            'rviz': 'false',
            'pointcloud_topic': '/livox/lidar',
            'imu_topic': '/livox/imu',
        }.items()
    )

    # 2. Launch Open3D SLAM after 10 seconds
    open3d_slam_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('open3d_slam_ros'), '/launch/open3d_launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'launch_rviz': 'false',
                    'cloud_topic': '/livox/lidar',
                }.items()
            )
        ]
    )

    # 3. Launch Vehicle Simulator after 15 seconds (10 + 5)
    vehicle_simulator_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    vehicle_simulator_pkg, '/launch/system_real_robot.launch'
                ])
            )
        ]
    )
    
    # 4. Launch Far Planner after 20 seconds (15 + 5)
    far_planner_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    far_planner_pkg, '/launch/far_planner.launch'
                ])
            )
        ]
    )

    # Foxglove Bridge launch (optional, starts immediately)
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            FindPackageShare('foxglove_bridge'), '/launch/foxglove_bridge_launch.xml'
        ])
    )
    
    return LaunchDescription([
        dlio_launch,           # 0s  - DLIO starts first
        open3d_slam_launch,    # 10s - Open3D SLAM
        vehicle_simulator_launch,  # 15s - Vehicle Simulator
        far_planner_launch,    # 20s - Far Planner
        # foxglove_bridge_launch,
    ])
