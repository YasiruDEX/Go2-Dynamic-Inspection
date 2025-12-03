#!/usr/bin/env python3
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
"""
Development Pipeline Launch File.

Launches the pipeline with minimal delays for rapid testing and
development iterations.

Launch Sequence:
    All components start with 1-second delays for quick startup.

Usage:
    ros2 launch pipeline_launcher pipeline_dev.launch.py
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
from pipeline_launcher_lib.config import QUICK_TIMING, TOPICS_DLIO_OUTPUT, TOPICS_LIVOX


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for development pipeline."""
    # Declare launch arguments
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Enable verbose logging",
    )

    # Use quick timing for development
    timing = QUICK_TIMING

    # Create launch actions
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
            declare_verbose,
            foxglove,
            dlio,
            open3d_slam,
            vehicle_simulator,
            far_planner,
        ]
    )
