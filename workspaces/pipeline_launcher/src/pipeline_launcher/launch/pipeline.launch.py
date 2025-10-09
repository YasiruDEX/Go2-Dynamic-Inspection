from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch the complete pipeline with sequential delays:
    1. fast_lio mapping_mid360.launch.py (starts immediately)
    2. vehicle_simulator system_real_robot.launch (after 3 seconds)
    3. far_planner far_planner.launch (after 6 seconds total)
    """
    
    # Find package paths
    fast_lio_pkg = FindPackageShare('fast_lio')
    vehicle_simulator_pkg = FindPackageShare('vehicle_simulator')
    far_planner_pkg = FindPackageShare('far_planner')
    
    # Launch fast_lio mapping immediately (with RViz disabled)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            fast_lio_pkg, '/launch/mapping_mid360.launch.py'
        ]),
        launch_arguments={
            'rviz': 'false'
        }.items()
    )
    
    # Launch vehicle_simulator after 3 seconds
    vehicle_simulator_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    vehicle_simulator_pkg, '/launch/system_real_robot.launch'
                ])
            )
        ]
    )

    # Launch dlio (assuming it is part of the pipeline)
    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('direct_lidar_inertial_odometry'), '/launch/dlio.launch.py'
        ]),
        launch_arguments={
            'rviz': 'false'
        }.items()
    )
    
    # Launch far_planner after 6 seconds total (3 more after vehicle_simulator)
    far_planner_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    far_planner_pkg, '/launch/far_planner.launch'
                ])
            )
        ]
    )
    
    return LaunchDescription([
        # fast_lio_launch,
        dlio_launch,
        vehicle_simulator_launch,
        far_planner_launch,
    ])
