# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            GroupAction, Shutdown, OpaqueFunction)
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():

    # ~~~~~~~~~~~~~~~~~~~~~~~~
    #  Smoother arguments
    # ~~~~~~~~~~~~~~~~~~~~~~~~
    navstate_kinematic_model_arg = DeclareLaunchArgument(
        "navstate_kinematic_model",
        default_value="KinematicModel::ConstantVelocity",
        description="[Smoother only] Kinematic model for internal motion model factors. Options: KinematicModel::ConstantVelocity, KinematicModel::Tricycle.")

    navstate_sliding_window_sec_arg = DeclareLaunchArgument(
        "navstate_sliding_window_sec",
        default_value="2.5",
        description="[Smoother only] Time window to keep past observations in the filter [seconds].")

    navstate_sigma_random_walk_linacc_arg = DeclareLaunchArgument(
        "navstate_sigma_random_walk_linacc",
        default_value="1.0",
        description="[Smoother only] Random walk model for linear acceleration uncertainty [m/s²].")

    navstate_sigma_random_walk_angacc_arg = DeclareLaunchArgument(
        "navstate_sigma_random_walk_angacc",
        default_value="10.0",
        description="[Smoother only] Random walk angular acceleration uncertainty [rad/s²].")

    estimate_geo_reference_arg = DeclareLaunchArgument(
        "estimate_geo_reference",
        default_value="False",
        description="[Smoother only] Whether to estimate the best geo-referencing for {enu} -> {map} from incoming GNSS readings.")

    # ~~~~~~~~~~~~~~~~~~~~~~~~
    # ros2bridge arguments
    # ~~~~~~~~~~~~~~~~~~~~~~~~
    ignore_lidar_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_lidar_pose_from_tf", default_value="false", description="If true, the LiDAR pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf")
    ignore_lidar_pose_from_tf_env_var = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_LIDAR_POSE', value=LaunchConfiguration('ignore_lidar_pose_from_tf'))

    ignore_imu_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_imu_pose_from_tf", default_value="false", description="If true, the IMU pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf")
    ignore_imu_pose_from_tf_env_var = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_IMU_POSE', value=LaunchConfiguration('ignore_imu_pose_from_tf'))

    gnss_topic_name_arg = DeclareLaunchArgument(
        "gnss_topic_name", default_value="gps", description="Topic name to listen for NavSatFix input from a GNSS (for example '/gps')")
    gps_topic_env_var = SetEnvironmentVariable(
        name='MOLA_GNSS_TOPIC', value=LaunchConfiguration('gnss_topic_name'))

    imu_topic_name_arg = DeclareLaunchArgument(
        "imu_topic_name", default_value="imu", description="Topic name to listen for Imu input (for example '/imu')")
    imu_topic_name_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_TOPIC', value=LaunchConfiguration('imu_topic_name'))

    use_mola_gui_arg = DeclareLaunchArgument(
        "use_mola_gui", default_value="True", description="Whether to open MolaViz GUI interface for watching live mapping and control UI")
    use_mola_gui_env_var = SetEnvironmentVariable(
        name='MOLA_WITH_GUI', value=LaunchConfiguration('use_mola_gui'))

    mola_se_reference_frame_arg = DeclareLaunchArgument(
        "mola_state_estimator_reference_frame", default_value="map", description="The /tf frame name to be used as reference for MOLA State Estimators to publish pose updates")
    mola_tf_map_env_var = SetEnvironmentVariable(
        name='MOLA_TF_MAP', value=LaunchConfiguration('mola_state_estimator_reference_frame'))

    mola_footprint_to_base_link_tf_arg = DeclareLaunchArgument(
        "mola_footprint_to_base_link_tf", default_value="[0, 0, 0, 0, 0, 0]", description="Transformation between base_footprint and base_link.")
    mola_footprint_to_base_link_tf_env_var = SetEnvironmentVariable(
        name='MOLA_TF_FOOTPRINT_TO_BASE_LINK', value=LaunchConfiguration('mola_footprint_to_base_link_tf'))

    enforce_planar_motion_arg = DeclareLaunchArgument(
        "enforce_planar_motion", default_value="False", description="Whether to enforce z, pitch, and roll to be zero.")
    enforce_planar_motion_env_var = SetEnvironmentVariable(
        name='MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION', value=LaunchConfiguration('enforce_planar_motion'))

    forward_ros_tf_odom_to_mola_arg = DeclareLaunchArgument(
        "forward_ros_tf_odom_to_mola", default_value="False", description="Whether to import an existing /tf 'odom'->'base_link' odometry.")
    forward_ros_tf_odom_to_mola_env_var = SetEnvironmentVariable(
        name='MOLA_FORWARD_ROS_TF_ODOM_TO_MOLA', value=LaunchConfiguration('forward_ros_tf_odom_to_mola'))

    # Environment variables
    smoother_env_vars = GroupAction(
        actions=[
            SetEnvironmentVariable('MOLA_NAVSTATE_KINEMATIC_MODEL', LaunchConfiguration(
                'navstate_kinematic_model')),
            SetEnvironmentVariable('MOLA_NAVSTATE_SLIDING_WINDOW_SEC', LaunchConfiguration(
                'navstate_sliding_window_sec')),
            SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC', LaunchConfiguration(
                'navstate_sigma_random_walk_linacc')),
            SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC', LaunchConfiguration(
                'navstate_sigma_random_walk_angacc')),
            SetEnvironmentVariable(
                'MOLA_ESTIMATE_GEO_REF', LaunchConfiguration('estimate_geo_reference')),
        ]
    )

    localization_publish_tf_source_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_TF_SOURCE',
        value='state_estimator')

    localization_publish_odom_source_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE',
        value='state_estimator')

    mola_tf_base_link_arg = DeclareLaunchArgument(
        "mola_tf_base_link", default_value="base_link",
        description="The /tf frame name for the robot base link")
    mola_tf_base_link_env_var = SetEnvironmentVariable(
        name='MOLA_TF_BASE_LINK', value=LaunchConfiguration('mola_tf_base_link'))

    # Namespace (Based on Nav2's bring-up launch file!)
    # ---------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    #
    # (JLBC further explanation) The problem is the "tf2" library. It's hardcoded to subscribe
    # to "/tf". This remapping allows "/robot/tf" to be seen as "/tf" so tf2_ros (and RViz) can see it.
    #
    tf_remaps = [('/tf', 'tf'),
                 ('/tf_static', 'tf_static')]

    # MOLA subsystem configuration YAML file
    # ------------------------------------------
    myDir = get_package_share_directory("mola_state_estimation_smoother")

    mola_system_yaml_file = os.path.join(
        myDir, 'mola-cli-launchs', 'state_estimator_ros2.yaml')

    # -------------------
    #        Node
    # -------------------
    node_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            package='mola_launcher',
            executable='mola-cli',
            output='screen',
            remappings=tf_remaps,
            arguments=[mola_system_yaml_file],
            on_exit=Shutdown()
        )
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        enforce_planar_motion_arg,
        enforce_planar_motion_env_var,
        forward_ros_tf_odom_to_mola_arg,
        forward_ros_tf_odom_to_mola_env_var,
        gnss_topic_name_arg,
        gps_topic_env_var,
        ignore_imu_pose_from_tf_arg,
        ignore_imu_pose_from_tf_env_var,
        ignore_lidar_pose_from_tf_arg,
        ignore_lidar_pose_from_tf_env_var,
        imu_topic_name_arg,
        imu_topic_name_env_var,
        mola_footprint_to_base_link_tf_arg,
        mola_footprint_to_base_link_tf_env_var,
        mola_se_reference_frame_arg,
        mola_tf_base_link_arg,
        mola_tf_base_link_env_var,
        mola_tf_map_env_var,
        use_mola_gui_arg,
        use_mola_gui_env_var,

        # Smoother Specific
        navstate_kinematic_model_arg,
        navstate_sliding_window_sec_arg,
        navstate_sigma_random_walk_linacc_arg,
        navstate_sigma_random_walk_angacc_arg,
        estimate_geo_reference_arg,
        smoother_env_vars,

        localization_publish_tf_source_env_var,
        localization_publish_odom_source_env_var,
        # group
        node_group
    ])
