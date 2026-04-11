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
 * @brief  Parameters for StateEstimationSimple
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>

namespace mola::state_estimation_simple
{
/** Parameters needed by StateEstimationSimple.
 *
 * \ingroup mola_state_estimation_grp
 */
class Parameters
{
   public:
    Parameters() = default;

    /// Loads all parameters from a YAML map node.
    void loadFrom(const mrpt::containers::yaml& cfg);

    /** Valid estimations will be extrapolated only up to this time since the
     * last incorporated observation. */
    double max_time_to_use_velocity_model = 2.0;  // [s]

    mrpt::math::TTwist3D initial_twist;

    double sigma_random_walk_acceleration_linear  = 1.0;  // [m/s²]
    double sigma_random_walk_acceleration_angular = 10.0;  // [rad/s²]

    double sigma_relative_pose_linear  = 1.0;  // [m]
    double sigma_relative_pose_angular = 0.1;  // [rad]

    double sigma_imu_angular_velocity = 0.05;  // [rad/s]

    bool enforce_planar_motion = false;

    /// regex for IMU sensor labels (ROS topics) to accept as IMU readings.
    std::string do_process_imu_labels_re = ".*";

    /// regex for odometry inputs labels (ROS topics) to be accepted as inputs
    std::string do_process_odometry_labels_re = ".*";

    /// regex for GNSS (GPS) labels (ROS topics) to be accepted as inputs
    std::string do_process_gnss_labels_re = ".*";
};

}  // namespace mola::state_estimation_simple
