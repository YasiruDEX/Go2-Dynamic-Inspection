#!/usr/bin/env python3
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
"""
Real Robot Pipeline Launch File.

Launches the complete autonomy pipeline for real robot operation with
Livox LiDAR sensors. Components are started with sequential delays to
ensure proper initialization.

Launch Sequence:
    1. Foxglove Bridge   (T=0s)  - Visualization bridge
    2. DLIO              (T=5s)  - LiDAR-Inertial Odometry
    3. Open3D SLAM       (T=15s) - 3D mapping
    4. Vehicle Simulator (T=20s) - Motion planning interface
    5. Far Planner       (T=25s) - Global path planning

Usage:
    ros2 launch pipeline_launcher pipeline_real.launch.py
    ros2 launch pipeline_launcher pipeline_real.launch.py dlio_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from pipeline_launcher_lib.launch_utils import (
    create_dlio_launch,
    create_far_planner_launch,
    create_foxglove_launch,
    create_open3d_slam_launch,
    create_vehicle_simulator_launch,
)
from pipeline_launcher_lib.config import DEFAULT_TIMING, TOPICS_DLIO_OUTPUT, TOPICS_LIVOX


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for real robot pipeline."""
    # Declare launch arguments
    declare_dlio_rviz = DeclareLaunchArgument(
        "dlio_rviz",
        default_value="true",
        description="Enable RViz visualization for DLIO",
    )

    declare_slam_rviz = DeclareLaunchArgument(
        "slam_rviz",
        default_value="false",
        description="Enable RViz visualization for Open3D SLAM",
    )

    # Create launch actions with configured timing
    timing = DEFAULT_TIMING

    foxglove = create_foxglove_launch(delay=timing.foxglove_bridge)

    dlio = create_dlio_launch(
        delay=timing.dlio,
        rviz="true",
        pointcloud_topic=TOPICS_LIVOX.pointcloud,
        imu_topic=TOPICS_LIVOX.imu,
    )

    open3d_slam = create_open3d_slam_launch(
        delay=timing.open3d_slam,
        use_sim_time="false",
        launch_rviz="false",
        cloud_topic=TOPICS_DLIO_OUTPUT.pointcloud,
    )

    vehicle_simulator = create_vehicle_simulator_launch(delay=timing.vehicle_simulator)

    far_planner = create_far_planner_launch(delay=timing.far_planner)

    return LaunchDescription(
        [
            # Launch argument declarations
            declare_dlio_rviz,
            declare_slam_rviz,
            # Launch actions (ordered by start time)
            foxglove,  # T=0s
            dlio,  # T=5s
            open3d_slam,  # T=15s
            vehicle_simulator,  # T=20s
            far_planner,  # T=25s
        ]
    )
