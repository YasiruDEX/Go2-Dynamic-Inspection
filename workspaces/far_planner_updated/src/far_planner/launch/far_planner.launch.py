import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression, PathJoinSubstitution
import launch_ros

def generate_launch_description():
    pkg_share = get_package_share_directory('far_planner')
    default_config = os.path.join(pkg_share, 'config', 'default.yaml')
    default_rviz   = os.path.join(pkg_share, 'rviz', 'default.rviz')

    return LaunchDescription([
        SetParameter(name='use_sim_time', value='false'),

        DeclareLaunchArgument(
            'vgraph_autoload',
            default_value='false',
            description='If true, FAR Planner will auto-load a saved visibility graph at startup.',
        ),
        DeclareLaunchArgument(
            'vgraph_file_path',
            default_value='',
            description='Absolute path to a .vgh file to load when vgraph_autoload is true.',
        ),

        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            parameters=[
                default_config,
                {
                    'vgraph_autoload': LaunchConfiguration('vgraph_autoload'),
                    'vgraph_file_path': LaunchConfiguration('vgraph_file_path'),
                },
            ],
            remappings=[
                ('/odom_world', '/lidar_odometry/pose'),
                ('/terrain_cloud', '/terrain_map_ext'),
                ('/terrain_local_cloud', '/terrain_map'),
                ('/scan_cloud', '/lidar_odometry/deskewed_scan_points')
            ]
        ),
        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='far_rviz',
        #     arguments=['-d', default_rviz],
        #     respawn=False,
        # )
    ])
