#!/usr/bin/env python3
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
"""
Mapping-Only Pipeline Launch File.

Launches only the odometry and mapping components without the planning
stack. Useful for data collection, map building, and sensor testing.

Launch Sequence:
    1. DLIO        (T=0s)  - LiDAR-Inertial Odometry
    2. Open3D SLAM (T=5s)  - 3D mapping

Usage:
    ros2 launch pipeline_launcher pipeline_mapping.launch.py
    ros2 launch pipeline_launcher pipeline_mapping.launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from pipeline_launcher_lib.launch_utils import (
    create_dlio_launch,
    create_foxglove_launch,
    create_open3d_slam_launch,
)
from pipeline_launcher_lib.config import TOPICS_DLIO_OUTPUT, TOPICS_LIVOX


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for mapping-only pipeline."""
    # Declare launch arguments
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Enable RViz visualization",
    )

    declare_foxglove = DeclareLaunchArgument(
        "foxglove",
        default_value="false",
        description="Enable Foxglove Bridge",
    )

    # Timing configuration
    DELAY_DLIO = 0.0
    DELAY_OPEN3D = 5.0

    # Create launch actions
    dlio = create_dlio_launch(
        delay=DELAY_DLIO,
        rviz="true",
        pointcloud_topic=TOPICS_LIVOX.pointcloud,
        imu_topic=TOPICS_LIVOX.imu,
    )

    open3d_slam = create_open3d_slam_launch(
        delay=DELAY_OPEN3D,
        use_sim_time="false",
        launch_rviz="true",
        cloud_topic=TOPICS_DLIO_OUTPUT.pointcloud,
    )

    return LaunchDescription(
        [
            declare_rviz,
            declare_foxglove,
            dlio,
            open3d_slam,
        ]
    )
