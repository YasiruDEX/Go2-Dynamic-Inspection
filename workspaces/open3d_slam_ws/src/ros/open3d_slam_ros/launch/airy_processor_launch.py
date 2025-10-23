import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'launch_prefix', default_value='',
            description='Prefix to launch node, e.g., "gdb -ex run --args"'
        ),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/rslidar_points',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'pose_topic', default_value='/state_estimator/pose_in_odom',
            description='Input pose topic'
        ),
        DeclareLaunchArgument(
            'distance_cutoff', default_value='0.2',
            description='Distance cutoff in meters'
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Whether to launch RViz visualization'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value='/opt/ros/jazzy/share/rviz2/rviz/default_config.rviz',
            description='Path to RViz default config file'
        ),

        # Airy Processing Node
        Node(
            package='open3d_slam_ros',
            executable='airy_processing_node',
            name='airy_processor',
            output='screen',
            parameters=[{
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'distance_cutoff': LaunchConfiguration('distance_cutoff'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),

        # RViz launch (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='airy_rviz',
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('launch_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])
