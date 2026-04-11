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
 * @file   StateEstimationSimple.h
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

// This package
#include <mola_state_estimation_simple/Parameters.h>
#include <mola_state_estimation_simple/RegexCache.h>

// MOLA
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/version.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <mutex>
#include <optional>

namespace mola::state_estimation_simple
{
/** Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 *
 * Usage:
 * - (1) Call initialize() or set the required parameters directly in params_.
 * - (2) Integrate measurements with `fuse_*()` methods. Each CObservation
 *       class includes a `timestamp` field which is used to estimate the
 *       trajectory.
 * - (3) Repeat (2) as needed.
 * - (4) Read the estimation up to any nearby moment in time with
 *       estimated_navstate()
 *
 * Old observations are automatically removed.
 *
 * \note This implementation of mola::NavStateFilter ignores the passed
 *       "frame_id".
 * \note It also ignore GNSS sensor.
 *
 * \sa mola::IMUIntegrator
 *
 * \ingroup mola_state_estimation_grp
 */
class StateEstimationSimple : public mola::NavStateFilter

{
    DEFINE_MRPT_OBJECT(StateEstimationSimple, mola::state_estimation_simple)

   public:
    StateEstimationSimple();

    /** \name Main API
     *  @{ */

    Parameters params;

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const mrpt::containers::yaml& cfg) override;

    void spinOnce() override;

    /** Resets the estimator state to an initial state.
     *  \sa currentIntegrationState
     */
    void reset() override;

    /** Integrates new SE(3) pose estimation of the vehicle wrt frame_id
     */
    void fuse_pose(
        const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string& frame_id) override;

    /** Integrates new wheels-based odometry observations into the estimator.
     *  This is a convenience method that internally ends up calling
     *  fuse_pose(), but computing the uncertainty of odometry increments
     *  according to a given motion model.
     */
    void fuse_odometry(
        const mrpt::obs::CObservationOdometry& odom,
        const std::string&                     odomName = "odom_wheels") override;

    /** Integrates new IMU observations into the estimator */
    void fuse_imu(const mrpt::obs::CObservationIMU& imu) override;

    /** Integrates new GNSS observations into the estimator */
    void fuse_gnss(const mrpt::obs::CObservationGPS& gps) override;

    /** Integrates new twist estimation (in the odom frame) */
    void fuse_twist(
        const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
        const mrpt::math::CMatrixDouble66& twistCov) override;

    /** Computes the estimated vehicle state at a given timestep using the
     * observations in the time window. A std::nullopt is returned if there is
     * no valid observations yet, or if requested a timestamp out of the model
     * validity time window (e.g. too far in the future to be trustful).
     */
    std::optional<NavState> estimated_navstate(
        const mrpt::Clock::time_point& timestamp, const std::string& frame_id) override;

    std::optional<mrpt::math::TTwist3D> get_last_twist() const;

    /** @} */

    // Implementation of RawDataConsumer
    void onNewObservation(const CObservation::ConstPtr& o) override;

   private:
    struct State
    {
        State() = default;

        std::optional<mrpt::obs::CObservationOdometry> last_odom_obs;
        std::optional<mrpt::Clock::time_point>         last_pose_obs_tim;
        std::optional<mrpt::poses::CPose3DPDFGaussian> last_pose;
        std::optional<mrpt::math::TTwist3D>            last_twist;
        std::optional<mrpt::math::CMatrixDouble66>     last_twist_cov;
        bool                                           pose_already_updated_with_odom = false;

        // To be built from parameters strings when changed.
        RegexCache do_process_imu_labels_re;
        RegexCache do_process_odometry_labels_re;
        RegexCache do_process_gnss_labels_re;
    };

    State                        state_;
    mutable std::recursive_mutex state_mtx_;
};

}  // namespace mola::state_estimation_simple
