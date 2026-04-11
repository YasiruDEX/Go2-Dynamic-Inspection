/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   trajectory_from_buffer.h
 * @brief  Reconstruct a trajectory from a LocalVelocityBuffer
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */

#pragma once

#include <mola_imu_preintegration/ImuIntegrationParams.h>
#include <mola_imu_preintegration/LocalVelocityBuffer.h>
#include <mola_imu_preintegration/types.h>

namespace mola::imu
{
/** Each of the recovered-trajectory key-points.
 * Some members are std::optional just to help in the process of reconstruction, to know which
 * are already computed.
 *
 * \ingroup mola_imu_preintegration_grp
 */
struct TrajectoryPoint
{
    TrajectoryPoint() = default;

    /// Constructor from a gravity-aligned rotation and a position
    TrajectoryPoint(const mrpt::math::CMatrixDouble33& R_, const mrpt::math::TVector3D& p_)
        : pose(mrpt::poses::CPose3D::FromRotationAndTranslation(
              R_, mrpt::math::CVectorFixedDouble<3>(p_)))
    {
    }

    /// SE(3) pose, relative to "t=0"
    mrpt::poses::CPose3D pose;

    /// SO(3) orientation, gravity aligned
    std::optional<mrpt::math::CMatrixDouble33> R_ga;

    /// Linear velocity (v) in the frame of reference of "t=0"
    std::optional<LinearVelocity> v;

    // Angular velocity (ω) in the body (b) frame (directly from IMU)
    std::optional<AngularVelocity> w_b;

    /// Linear acceleration (a) in the body (b) frame (directly from IMU, transformed)
    /// (This one still has gravity)
    std::optional<LinearAcceleration> a_b;

    /// Linear coordinate acceleration (body frame) (without gravity effects)
    std::optional<LinearAcceleration> ac_b;

    /// Angular acceleration (α)
    AngularAcceleration alpha_b = {0, 0, 0};

    /// Jerk = \dot{a}
    mrpt::math::TVector3D j_b = {0, 0, 0};

    std::string asString() const;
};

/// A recovered trajectory, indexed by relative time in seconds (t=0 is the scan reference
/// stamp)
using Trajectory = std::map<double, TrajectoryPoint>;

/// Print a trajectory to a human-readable string (mostly for debugging)
std::string trajectory_as_string(const Trajectory& traj);

/**
 * @brief Reconstruct a trajectory from a LocalVelocityBuffer with IMU data.
 *
 * The exact reconstruction algorithm is described in the paper [TBD], but in short,
 * it consists of integrating the IMU data using as anchor at least one global gravity-aligned
 * orientation and one linear velocity.
 *
 * @param samples IMU and other LIO data samples
 * @param imu_params IMU integration parameters, in particular, biases
 * @param use_higher_order Whether to use higher-order integration (jerk)
 * @return Trajectory With the reconstructed trajectory
 */
Trajectory trajectory_from_buffer(
    const LocalVelocityBuffer::SampleHistory& samples, const ImuIntegrationParams& imu_params,
    bool use_higher_order = false);

}  // namespace mola::imu
