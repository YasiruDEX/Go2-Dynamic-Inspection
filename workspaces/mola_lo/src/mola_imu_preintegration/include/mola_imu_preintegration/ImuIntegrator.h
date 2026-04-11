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
 * @file   ImuIntegrator.h
 * @brief  Integrator of IMU accelerations and angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */
#pragma once

#include <mola_imu_preintegration/ImuIntegrationParams.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>

namespace mola::imu
{
/** Integrates acceleration and gyroscope readings.
 *
 * Bias is assumed to be constant during the short periods this preintegrator is expected to be
 * used.
 *
 * See ImuIntegrationParams for a list of related papers explaining the methods
 * and parameters.
 *
 * Usage:
 * - (1) Call initialize() or set the required parameters directly in params_.
 * - (2) Integrate measurements with integrate_measurement()
 * - (3) Repeat (2) N times as needed.
 * - (4) Take the estimation up to this point with current_integration_state()
 *       and reset with reset_integration() if you create a new key-frame.
 * - (5) Go to (2).
 *
 * \note Initially based in part on GTSAM sources gtsam::PreintegratedRotation.
 * \note Based on Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2015). IMU
 *       preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation.
 *
 * \sa IMUIntegrator
 * \ingroup mola_imu_preintegration_grp
 */
class ImuIntegrator
{
   public:
    ImuIntegrator() = default;

    struct IntegrationState
    {
        IntegrationState() = default;

        /// Time interval from i to j
        double deltaTij = 0;

        /// Preintegrated relative orientation (in frame i)
        mrpt::math::CMatrixDouble33 deltaRij = mrpt::math::CMatrixDouble33::Identity();

        /// Jacobian of preintegrated rotation w.r.t. angular rate bias
        // (TODO)
        // mrpt::math::CMatrixDouble33 delRdelBiasOmega_ =
        // mrpt::math::CMatrixDouble33::Zero();
    };

    /** \name Main API
     *  @{ */

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from, as
     * expected by RotationIntegrationParams (see its docs).
     */
    void initialize(const mrpt::containers::yaml& cfg);

    /** Resets the integrator state to an initial state.
     *  \sa currentIntegrationState
     */
    void reset_integration();

    const IntegrationState& current_integration_state() const { return state_; }

    /** Accumulates a new gyroscope measurement of the angular velocity (ω) into
     * the current preintegration state, integrating it forward in
     * time for a period dt [s].
     *
     * \sa current_integration_state(), reset_integration()
     */
    void integrate_measurement(const mrpt::math::TVector3D& w, double dt);

    /// All public parameters for the integrator, including biases
    ImuIntegrationParams parameters;

    /** @} */

   private:
    IntegrationState state_;
};

/**
 * This takes a gyroscope measurement of the angular
 * velocity (ω) and integrates it forward in time for a period dt [s] as:
 *
 *  Rot = Exp((ω-ω_{bias})·dt)
 *
 * \ingroup mola_imu_preintegration_grp
 */
mrpt::math::CMatrixDouble33 incremental_rotation(
    const mrpt::math::TVector3D& w, const ImuIntegrationParams& params, double dt,
    const mrpt::optional_ref<mrpt::math::CMatrixDouble33>& D_incrR_integratedOmega = std::nullopt);

}  // namespace mola::imu
