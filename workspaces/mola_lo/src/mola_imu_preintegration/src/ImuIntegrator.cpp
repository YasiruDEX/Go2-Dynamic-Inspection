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
 * @file   ImuIntegrator.cpp
 * @brief  Integrator of IMU accelerations and angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */

#include <mola_imu_preintegration/ImuIntegrator.h>
#include <mrpt/poses/Lie/SO.h>

using namespace mola::imu;

void ImuIntegrator::initialize(const mrpt::containers::yaml& cfg)
{
    reset_integration();

    // Load params:
    parameters.load_from(cfg);
}

void ImuIntegrator::reset_integration()
{
    // reset:
    state_ = IntegrationState();
}

void ImuIntegrator::integrate_measurement(const mrpt::math::TVector3D& w, double dt)
{
    const auto incrR = mola::imu::incremental_rotation(w, parameters, dt);

    // Update integration state:
    state_.deltaTij += dt;
    state_.deltaRij = state_.deltaRij * incrR;

    // TODO: Update Jacobian
}

mrpt::math::CMatrixDouble33 mola::imu::incremental_rotation(
    const mrpt::math::TVector3D& w, const ImuIntegrationParams& params, double dt,
    const mrpt::optional_ref<mrpt::math::CMatrixDouble33>& D_incrR_integratedOmega)
{
    using mrpt::math::TVector3D;

    // Bias:
    TVector3D correctedW = w - params.bias_gyro;

    // Translate to vehicle frame:
    if (params.sensor_pose.has_value())
    {
        correctedW = params.sensor_pose->rotateVector(correctedW);
    }

    // Integrate:
    const TVector3D w_dt = correctedW * dt;

    if (D_incrR_integratedOmega.has_value())
    {
        // TODO: Jacobian: mrpt::poses::Lie::SO<3>::jacob_dexpe_de()
        THROW_EXCEPTION("Jacobian not implemented yet");
    }

    return mrpt::poses::Lie::SO<3>::exp(mrpt::math::CVectorFixedDouble<3>(w_dt));
}
