#!/usr/bin/env python3
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
"""
Simulation Pipeline Launch File.

Launches the complete autonomy pipeline for Gazebo simulation with
Velodyne LiDAR sensors.

Launch Sequence:
    1. Foxglove Bridge   (T=0s)  - Visualization bridge
    2. DLIO              (T=3s)  - LiDAR-Inertial Odometry
    3. Vehicle Simulator (T=6s)  - Motion planning interface
    4. Far Planner       (T=9s)  - Global path planning
    5. Open3D SLAM       (T=9s)  - 3D mapping

Usage:
    ros2 launch pipeline_launcher pipeline_simulation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from pipeline_launcher_lib.launch_utils import (
    create_dlio_launch,
    create_far_planner_launch,
    create_foxglove_launch,
    create_open3d_slam_launch,
    create_vehicle_simulator_launch,
)
from pipeline_launcher_lib.config import TOPICS_VELODYNE


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for simulation pipeline."""
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time from Gazebo",
    )

    # Timing configuration for simulation
    DELAY_FOXGLOVE = 0.0
    DELAY_DLIO = 3.0
    DELAY_VEHICLE = 6.0
    DELAY_FAR_PLANNER = 9.0
    DELAY_OPEN3D = 9.0

    # Create launch actions
    foxglove = create_foxglove_launch(delay=DELAY_FOXGLOVE)

    dlio = create_dlio_launch(
        delay=DELAY_DLIO,
        rviz="false",
        pointcloud_topic=TOPICS_VELODYNE.pointcloud,
        imu_topic=TOPICS_VELODYNE.imu,
    )

    vehicle_simulator = create_vehicle_simulator_launch(delay=DELAY_VEHICLE)

    far_planner = create_far_planner_launch(delay=DELAY_FAR_PLANNER)

    open3d_slam = create_open3d_slam_launch(
        delay=DELAY_OPEN3D,
        use_sim_time="true",
        launch_rviz="false",
        cloud_topic=TOPICS_VELODYNE.pointcloud,
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            foxglove,
            dlio,
            vehicle_simulator,
            far_planner,
            open3d_slam,
        ]
    )
