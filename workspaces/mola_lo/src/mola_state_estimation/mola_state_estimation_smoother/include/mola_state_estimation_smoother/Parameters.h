/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   Parameters.h
 * @brief  Parameters for StateEstimationSmoother
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mola::state_estimation_smoother
{

enum class KinematicModel : uint8_t
{
    ConstantVelocity,
    Tricycle,
};

/** Parameters needed by StateEstimationSmoother.
 *
 * \ingroup mola_navstate_fuse__grp
 */
class Parameters
{
   public:
    Parameters() = default;

    /// Loads all parameters from a YAML map node.
    void loadFrom(const mrpt::containers::yaml& cfg);

    /** @name Reference frame IDs
     * @{  */

    /// Used to publish timely pose updates
    std::string vehicle_frame_name = "base_link";

    /// Used to publish timely pose updates. Typically, 'map' or 'odom', etc.
    /// See the docs online.
    std::string reference_frame_name = "map";

    /// The ENU geo-reference frame. See the docs online.
    std::string enu_frame_name = "enu";

    /** @}  */

    /** @name Kinematic factors and keyframe creation (motion model)
     * @{ */

    /** Kinematic model to be used in the internal motion model factors.
     *  Options: `KinematicModel::ConstantVelocity`, `KinematicModel::Tricycle`
     */
    KinematicModel kinematic_model = KinematicModel::ConstantVelocity;

    /** Valid estimations will be extrapolated only up to this time since the
     * last incorporated observation. If a request is done farther away, an
     * empty estimation will be returned.
     */
    double max_time_to_use_velocity_model = 2.0;  // [s]

    /// Time to keep past observations in the filter
    double sliding_window_length = 5.0;  // [s]

    double min_time_difference_to_create_new_frame = 0.01;  // [s]

    /// If the time between two keyframes is larger than this, a warning will be
    /// emitted; but the algorithm will keep trying its best.
    double time_between_frames_to_warning = 3.0;  // [s]

    /** When adding GNSS observations, specially with consumer grade receivers with errors larger
     * than a few centimeters, we may be more permissive in the temporal distance between the GNSS
     * datum and the associated existing keyframe. This parameter is the extended, alternative value
     * to use instead of "min_time_difference_to_create_new_frame". [seconds]
     */
    double gnss_nearby_keyframe_stamp_tolerance = 1.0;  // [s]

    /** When adding IMU observations, this is the temporal distance between the IMU reading
     * and the associated existing keyframe. This applies to gravity-oriented (IMU attitude) and
     * gravity-estimation (accelerometer) only factors, not to high-frequency IMU preintegration.
     *
     * This parameter is the extended, alternative value
     * to use instead of "min_time_difference_to_create_new_frame". [seconds]
     */
    double imu_nearby_keyframe_stamp_tolerance = 0.10;  // [s]

    double sigma_random_walk_acceleration_linear  = 1.0;  // [m/s²]
    double sigma_random_walk_acceleration_angular = 1.0;  // [rad/s²]
    double sigma_integrator_position              = 0.10;  // [m]
    double sigma_integrator_orientation           = 0.10;  // [rad]

    double sigma_twist_from_consecutive_poses_linear  = 1.0;  // [m/s]
    double sigma_twist_from_consecutive_poses_angular = 1.0;  // [rad/s]

    mrpt::math::TTwist3D initial_twist;

    // Defaults: somewhat confident that the vehicle is near rest.
    // Change these if needed to start with the vehicle at high speed.
    double initial_twist_sigma_lin = 0.1;  // [m/s]
    double initial_twist_sigma_ang = 0.1;  // [rad/s]

    bool enforce_planar_motion = false;

    /** If set, the first ever frame will also have an SE(3) edge favoring it to be the identity in
     * the "reference_frame", with a sigma given by this value. Use a small number, like 1e-6, for
     * initialing the first odometry pose near the map origin. Do not set when using geo-referenced
     * maps.
     */
    std::optional<double> link_first_pose_to_reference_origin_sigma;

    /** @} */

    /** @name IMU related
     * @{  */

    /** When an IMU provides global attitude measurements (azimuth and gravity aligned), this is the
     * uncertainty or noise sigma [degrees]. */
    double imu_attitude_sigma_deg = 2.0;

    /** When an IMU provides global attitude measurements (azimuth and gravity aligned), this must
     * define the angle (in degrees) to add to IMU yaw orientation to obtain azimuth so 0 deg is
     * North. Note that ENU axes are such vehicle yaw is 0 when pointing East instead.
     * Example cases:
     * - IMU absolute yaw=0 points True North ==> offset=0
     * - IMU absolute yaw=0 points East ==> offset=-90
     */
    double imu_attitude_azimuth_offset_deg = 0.0;

    /** When using an IMU with acceleration, use this sigma to estimate the up-vector, hence
     * gravity-align the map.
     * Set to 0 to disable.
     */
    double imu_normalized_gravity_alignment_sigma = 0.4;

    /** @} */

    /** @name Geo-referencing
     * @{  */

    /** If `true`, this estimator will try to estimate the best geo-referencing for {enu} ->
     * {map} from incoming GNSS readings and other sensors. If `false`, geo-referencing is
     * assumed to be given from either these initial parameters or, if not set, from an external
     * source (e.g. a geo-referenced `.mm` map loaded in mola_lidar_odometry).
     */
    bool estimate_geo_reference = false;

    /** If estimate_geo_reference is `false` and this is set, the geo-referencing will be taken
     * from this value and never attempted to be optimized or changed.
     * Other geo-reference information coming from external sources may override this fixed initial
     * value, though.
     */
    std::optional<mola::Georeferencing> fixed_geo_reference;

    /** Maximum position sigma (in meters) to consider state estimation converged.
     *  Used when other modules query `has_converged_localization()`.
     *  This applies to both, the `map->base_link` and `enu->map` poses.
     */
    double convergence_max_position_sigma = 1.0;  // [m]

    /** Maximum orientation sigma (in degrees) to consider state estimation converged.
     *  Used when other modules query `has_converged_localization()`.
     *  This applies to both, the `map->base_link` and `enu->map` poses.
     */
    double convergence_max_orientation_sigma_deg = 5.0;  // [deg]

    /** If true and estimate_geo_reference is true, once converged, this module
     *  will publish the estimated geo-referencing via MapSourceBase.
     */
    bool publish_estimated_georef_on_convergence = true;

    /** @} */

    /** @name Nonlinear optimization
     * @{ */

    /** Each new sensor will become a call to isam2.update(), plus this number of additional
     * refining steps. In theory, more steps lead to more accurate results. */
    uint32_t additional_isam2_update_steps = 3;

    /** @} */

    /** @name Sensor input names
     * @{  */

    /// regex for IMU sensor labels (ROS topics) to accept as IMU readings.
    std::string do_process_imu_labels_re = ".*";

    /// regex for odometry inputs labels (ROS topics) to be accepted as inputs
    std::string do_process_odometry_labels_re = ".*";

    /// regex for GNSS (GPS) labels (ROS topics) to be accepted as inputs
    std::string do_process_gnss_labels_re = ".*";

    /** @} */

    struct Visualization
    {
        bool show_console_messages = true;

        // this is automatically called by parent's loadFrom()
        void loadFrom(const mrpt::containers::yaml& cfg);
    };
    Visualization visualization;
};

}  // namespace mola::state_estimation_smoother

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(
    mola::state_estimation_smoother, mola::state_estimation_smoother::KinematicModel)
MRPT_FILL_ENUM(KinematicModel::ConstantVelocity);
MRPT_FILL_ENUM(KinematicModel::Tricycle);
MRPT_ENUM_TYPE_END()
