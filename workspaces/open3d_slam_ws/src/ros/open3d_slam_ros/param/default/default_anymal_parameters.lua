include "default/default_parameters.lua"

-- Clone the default parameter structure
DEFAULT_ANYMAL_PARAMETERS = {}
DEFAULT_ANYMAL_PARAMETERS.params = deepcopy(DEFAULT_PARAMETERS)
local params = DEFAULT_ANYMAL_PARAMETERS.params

----------------------------------------------------------------------
--                        GENERAL CONFIGURATION
----------------------------------------------------------------------

-- Use odometry topic instead of scan-to-scan registration
params.odometry.use_odometry_topic_instead_of_scan_to_scan = true

-- Disable IMU-based initialization of orientation
params.odometry.use_IMU_for_attitude_initialization = false

-- Disable dense mapping (used if you only want sparse maps)
params.mapper_localizer.is_build_dense_map = false

-- Republish a preloaded map (for visualization or loop closure)
params.mapper_localizer.republish_the_preloaded_map = true

-- Suppress internal timing logs
params.mapper_localizer.is_print_timing_information = false

-- Delay before merging new data into map (helps control timing/latency)
params.mapper_localizer.map_merge_delay_in_seconds = 10.0

-- Remove points through ray-casting (e.g. dynamic objects in scene)
params.mapper_localizer.is_carving_enabled = false
params.dense_map_builder.space_carving.carve_space_every_n_scans = 10
params.dense_map_builder.space_carving.truncation_distance = 0.1

-- Translation required between mapping steps
params.mapper_localizer.min_movement_between_mapping_steps = 0.0 -- meters

----------------------------------------------------------------------
--                        SCAN-TO-SCAN ODOMETRY
----------------------------------------------------------------------

-- Voxel grid filter for scan preprocessing
params.odometry.scan_processing.voxel_size = 0.05
params.odometry.scan_processing.downsampling_ratio = 1.0

----------------------------------------------------------------------
--                        MAP INITIALIZATION
----------------------------------------------------------------------

-- Disable merging scans into the map initially
params.mapper_localizer.is_merge_scans_into_map = false

-- Location of initial PCD map (for bootstrapping)
params.map_initializer.pcd_file_package = "rsl_seam_examples"
params.map_initializer.pcd_file_path = "/data/<mission>.pcd"

-- Start pose for map initialization (manual/static)
params.map_initializer.is_initialize_interactively = true
params.map_initializer.init_pose.x = 0.0
params.map_initializer.init_pose.y = 0.0
params.map_initializer.init_pose.z = 0.0
params.map_initializer.init_pose.roll = 0.0
params.map_initializer.init_pose.pitch = 0.0
params.map_initializer.init_pose.yaw = 0.0

-- Whether to use the map initializer output at all
params.mapper_localizer.is_use_map_initialization = false

----------------------------------------------------------------------
--                        DENSE MAP BUILDER
----------------------------------------------------------------------

-- Size of voxels in dense map
params.dense_map_builder.map_voxel_size = 0.05

-- Limit scan radius before carving/processing
params.dense_map_builder.scan_cropping.cropping_radius_max = 105.0

----------------------------------------------------------------------
--                        ICP REGISTRATION SETTINGS
----------------------------------------------------------------------

-- ICP registration tuning
params.mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 1.0
params.mapper_localizer.scan_to_map_registration.icp.knn = 10
params.mapper_localizer.scan_to_map_registration.icp.max_distance_knn = 1.0

-- No downsampling before ICP registration
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 1.0

-- Scan cropping radius during registration (avoid unnecessary cropping)
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 60.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 0.5

-- Voxel size for ICP registration
params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.15

params.mapper_localizer.scan_to_map_registration.icp.reference_cloud_seting_period = 2.0 --sec

----------------------------------------------------------------------
--                        MAP SAVING OPTIONS
----------------------------------------------------------------------

-- Save final map to disk
params.saving.save_map = true

-- Do not save individual submaps
params.saving.save_submaps = true

----------------------------------------------------------------------
--                        PLACE RECOGNITION
----------------------------------------------------------------------

-- RANSAC filtering and ICP refinement
params.place_recognition.ransac_min_corresondence_set_size = 40
params.place_recognition.max_icp_correspondence_distance = 0.3
params.place_recognition.min_icp_refinement_fitness = 0.7

-- Loop closure constraints
params.place_recognition.min_submaps_between_loop_closures = 2
params.place_recognition.loop_closure_search_radius = 50.0
params.mapper_localizer.is_attempt_loop_closures = false

-- Disable dumping of aligned loop closures
params.place_recognition.dump_aligned_place_recognitions_to_file = false

-- Pose consistency checks for loop closure validation
params.place_recognition.consistency_check.max_drift_roll = 30.0     -- deg
params.place_recognition.consistency_check.max_drift_pitch = 30.0    -- deg
params.place_recognition.consistency_check.max_drift_yaw = 30.0      -- deg
params.place_recognition.consistency_check.max_drift_x = 80.0        -- meters
params.place_recognition.consistency_check.max_drift_y = 80.0        -- meters
params.place_recognition.consistency_check.max_drift_z = 40.0        -- meters

----------------------------------------------------------------------
--                        SUBMAP PARAMETERS
----------------------------------------------------------------------

-- Allow up to 5 million points per submap
params.submap.max_num_points = 5000000

-- Avoid recomputing features too frequently
params.submap.min_seconds_between_feature_computation = 5.0

-- Allow revisiting a submap if fitness score is reasonable
params.submap.adjacency_based_revisiting_min_fitness = 0.5

-- Submap size
params.submap.submap_size = 10.0 --meters

-- Submap scan overlap
params.submap.submaps_num_scan_overlap = 10

----------------------------------------------------------------------
--                        MAP BUILDER CROPPING
----------------------------------------------------------------------

params.map_builder.scan_cropping.cropping_radius_max = 60.0
params.map_builder.scan_cropping.cropping_radius_min = 2.0

-- Map voxel size
params.map_builder.map_voxel_size = 0.2

-- Periodic carving of free space
params.map_builder.space_carving.carve_space_every_n_scans = 10

----------------------------------------------------------------------
--                        FINAL RETURN
----------------------------------------------------------------------

return params
