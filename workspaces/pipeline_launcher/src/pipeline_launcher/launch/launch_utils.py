#!/usr/bin/env python3
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
"""
Pipeline Launcher Utility Functions.

Provides helper functions to create standardized launch actions for
pipeline components. These utilities reduce code duplication and ensure
consistent configuration across all pipeline variants.
"""

from typing import Dict, List, Optional

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.substitutions import FindPackageShare

# Handle both installed package and development imports
try:
    from pipeline_launcher_lib import config
except ImportError:
    try:
        from . import config
    except ImportError:
        import config


def create_timed_launch(
    package: str,
    launch_file: str,
    delay: float,
    launch_arguments: Optional[Dict[str, str]] = None,
    use_xml: bool = False,
) -> TimerAction:
    """
    Create a TimerAction that launches a package after a specified delay.

    Args:
        package: Name of the ROS 2 package.
        launch_file: Path to launch file relative to package share directory.
        delay: Delay in seconds before launching.
        launch_arguments: Optional dictionary of launch arguments.
        use_xml: If True, use FrontendLaunchDescriptionSource for XML launch files.

    Returns:
        TimerAction configured to launch the specified package.
    """
    pkg_share = FindPackageShare(package)

    if use_xml:
        source = FrontendLaunchDescriptionSource([pkg_share, launch_file])
    else:
        source = PythonLaunchDescriptionSource([pkg_share, launch_file])

    include_action = IncludeLaunchDescription(
        source,
        launch_arguments=launch_arguments.items() if launch_arguments else None,
    )

    return TimerAction(period=delay, actions=[include_action])


def create_immediate_launch(
    package: str,
    launch_file: str,
    launch_arguments: Optional[Dict[str, str]] = None,
    use_xml: bool = False,
) -> IncludeLaunchDescription:
    """
    Create an IncludeLaunchDescription for immediate launch.

    Args:
        package: Name of the ROS 2 package.
        launch_file: Path to launch file relative to package share directory.
        launch_arguments: Optional dictionary of launch arguments.
        use_xml: If True, use FrontendLaunchDescriptionSource for XML launch files.

    Returns:
        IncludeLaunchDescription configured to launch the specified package.
    """
    pkg_share = FindPackageShare(package)

    if use_xml:
        source = FrontendLaunchDescriptionSource([pkg_share, launch_file])
    else:
        source = PythonLaunchDescriptionSource([pkg_share, launch_file])

    return IncludeLaunchDescription(
        source,
        launch_arguments=launch_arguments.items() if launch_arguments else None,
    )


# =============================================================================
# Pre-configured Launch Action Factories
# =============================================================================


def create_foxglove_launch(delay: float = 0.0) -> IncludeLaunchDescription | TimerAction:
    """Create Foxglove Bridge launch action."""
    if delay <= 0.0:
        return create_immediate_launch(
            package=config.PACKAGE_FOXGLOVE_BRIDGE,
            launch_file=config.LAUNCH_FOXGLOVE_BRIDGE,
            use_xml=True,
        )
    return create_timed_launch(
        package=config.PACKAGE_FOXGLOVE_BRIDGE,
        launch_file=config.LAUNCH_FOXGLOVE_BRIDGE,
        delay=delay,
        use_xml=True,
    )


def create_dlio_launch(
    delay: float,
    rviz: str = "false",
    pointcloud_topic: str = config.TOPICS_LIVOX.pointcloud,
    imu_topic: str = config.TOPICS_LIVOX.imu,
) -> TimerAction | IncludeLaunchDescription:
    """Create DLIO launch action with configurable topics."""
    args = config.DlioArgs(
        rviz=rviz,
        pointcloud_topic=pointcloud_topic,
        imu_topic=imu_topic,
    )

    if delay <= 0.0:
        return create_immediate_launch(
            package=config.PACKAGE_DLIO,
            launch_file=config.LAUNCH_DLIO,
            launch_arguments=args.to_dict(),
        )
    return create_timed_launch(
        package=config.PACKAGE_DLIO,
        launch_file=config.LAUNCH_DLIO,
        delay=delay,
        launch_arguments=args.to_dict(),
    )


def create_open3d_slam_launch(
    delay: float,
    use_sim_time: str = "false",
    launch_rviz: str = "false",
    cloud_topic: str = config.TOPICS_DLIO_OUTPUT.pointcloud,
) -> TimerAction:
    """Create Open3D SLAM launch action."""
    args = config.Open3dSlamArgs(
        use_sim_time=use_sim_time,
        launch_rviz=launch_rviz,
        cloud_topic=cloud_topic,
    )

    return create_timed_launch(
        package=config.PACKAGE_OPEN3D_SLAM,
        launch_file=config.LAUNCH_OPEN3D_SLAM,
        delay=delay,
        launch_arguments=args.to_dict(),
    )


def create_vehicle_simulator_launch(delay: float) -> TimerAction:
    """Create Vehicle Simulator launch action."""
    return create_timed_launch(
        package=config.PACKAGE_VEHICLE_SIMULATOR,
        launch_file=config.LAUNCH_VEHICLE_SIMULATOR,
        delay=delay,
    )


def create_far_planner_launch(delay: float) -> TimerAction:
    """Create Far Planner launch action."""
    return create_timed_launch(
        package=config.PACKAGE_FAR_PLANNER,
        launch_file=config.LAUNCH_FAR_PLANNER,
        delay=delay,
    )


def create_fast_lio_launch(
    delay: float = 0.0,
    rviz: str = "false",
) -> IncludeLaunchDescription | TimerAction:
    """Create Fast-LIO launch action."""
    args = config.FastLioArgs(rviz=rviz)

    if delay <= 0.0:
        return create_immediate_launch(
            package=config.PACKAGE_FAST_LIO,
            launch_file=config.LAUNCH_FAST_LIO,
            launch_arguments=args.to_dict(),
        )
    return create_timed_launch(
        package=config.PACKAGE_FAST_LIO,
        launch_file=config.LAUNCH_FAST_LIO,
        delay=delay,
        launch_arguments=args.to_dict(),
    )
