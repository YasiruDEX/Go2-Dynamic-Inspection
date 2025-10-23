import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# Preflight package presence check (prints colorful warning if missing)
def check_required_packages(pkgs):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    missing = []
    for pkg in pkgs:
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            missing.append(pkg)
    if missing:
        warn = (
            "\033[1;31m"  # bold red
            + "WARNING: Required ROS 2 packages missing: "
            + ", ".join(missing)
            + "\nInstall them with:\n    sudo apt install ros-${ROS_DISTRO}-"
            + " ros-${ROS_DISTRO}-".join(missing)
            + "\033[0m"
        )
        print(warn, file=sys.stderr)
    return missing

def generate_launch_description():
    check_required_packages([
        "open3d_slam_ros",
        "smb_description",
        "robot_state_publisher",
        "joint_state_publisher",
    ])

    launch_dir = os.path.dirname(os.path.realpath(__file__))

    declared_args = [
        DeclareLaunchArgument('cloud_topic', default_value='/rslidar/points'),
        DeclareLaunchArgument('odometry_topic', default_value='/graph_msf/est_odometry_odom_imu'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='empty'),
        DeclareLaunchArgument('launch_prefix', default_value=''),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('distance_cutoff', default_value='0.2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='/opt/ros/jazzy/share/rviz2/rviz/default_config.rviz'),
    ]

    description_file = PathJoinSubstitution(
        [FindPackageShare("smb_description"), "urdf", "smb.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

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
            robot_state_publisher_node,
            joint_state_publisher_node,
            open3d_launch,
        ]
    )
