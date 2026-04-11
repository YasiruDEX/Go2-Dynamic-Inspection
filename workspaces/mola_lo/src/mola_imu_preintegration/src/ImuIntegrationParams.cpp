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
 * @file   ImuIntegrationParams.cpp
 * @brief  Parameters for IMU preintegration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */

#include <mola_imu_preintegration/ImuIntegrationParams.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/TPoint3D.h>

using namespace mola::imu;

void ImuIntegrationParams::load_from(const mrpt::containers::yaml& cfg)
{
    if (cfg.has("bias_gyro"))
    {
        bias_gyro = mrpt::math::TVector3D::FromVector(cfg["bias_gyro"].toStdVector<double>());
    }
    if (cfg.has("bias_acc"))
    {
        bias_acc = mrpt::math::TVector3D::FromVector(cfg["bias_acc"].toStdVector<double>());
    }

    if (cfg.has("sensorLocationInVehicle"))
    {
        const auto cfgPose = cfg["sensorLocationInVehicle"];

        const auto poseTrans = cfgPose["translation"].toStdVector<double>();
        ASSERT_EQUAL_(poseTrans.size(), 3U);

        mrpt::poses::CPose3D pose;

        ASSERTMSG_(
            cfgPose.has("quaternion") || cfgPose.has("ypr_deg"),
            "'sensorLocationInVehicle' entry must contain either 'quaternion' or 'ypr_deg'");

        if (cfgPose.has("quaternion"))
        {
            const auto poseQuat = cfgPose["quaternion"].toStdVector<double>();
            ASSERT_EQUAL_(poseQuat.size(), 4U);

            pose = mrpt::poses::CPose3D::FromQuaternionAndTranslation(
                mrpt::math::CQuaternionDouble(poseQuat[3], poseQuat[0], poseQuat[1], poseQuat[2]),
                mrpt::math::TPoint3D::FromVector(poseTrans));
        }
        else
        {
            const auto poseYPR = cfgPose["ypr_deg"].toStdVector<double>();
            ASSERT_EQUAL_(poseYPR.size(), 3U);

            pose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                poseTrans[0], poseTrans[1], poseTrans[2], mrpt::DEG2RAD(poseYPR[0]),
                mrpt::DEG2RAD(poseYPR[1]), mrpt::DEG2RAD(poseYPR[2]));
        }

        if (pose != mrpt::poses::CPose3D::Identity())
        {
            // Store:
            sensor_pose = pose;
        }
        else
        {
            // Leave as unassigned to reflect it's just I_{4,4}
        }
    }

    if (cfg.has("gravity_vector"))
    {
        gravity_vector =
            mrpt::math::TVector3D::FromVector(cfg["gravity_vector"].toStdVector<double>());
    }

    if (cfg.has("cov_gyro"))
    {
        cfg["cov_gyro"].toMatrix(cov_gyro);
    }

    if (cfg.has("cov_acc"))
    {
        cfg["cov_acc"].toMatrix(cov_acc);
    }
}

namespace
{
mrpt::containers::yaml vectorToYaml(const mrpt::math::TVector3D& v)
{
    return mrpt::containers::yaml::Sequence({v.x, v.y, v.z});
}
}  // namespace

void ImuIntegrationParams::save_to(mrpt::containers::yaml& cfg) const
{
    cfg["gravity_vector"] = vectorToYaml(gravity_vector);
    cfg["bias_gyro"]      = vectorToYaml(bias_gyro);
    cfg["bias_acc"]       = vectorToYaml(bias_acc);

    cfg["cov_gyro"] = mrpt::containers::yaml::FromMatrix(cov_gyro);
    cfg["cov_acc"]  = mrpt::containers::yaml::FromMatrix(cov_acc);

    if (sensor_pose)
    {
        auto cfgPose           = cfg["sensorLocationInVehicle"];
        cfgPose["translation"] = vectorToYaml(sensor_pose->translation());
        cfgPose["ypr_deg"]     = vectorToYaml(
                {mrpt::RAD2DEG(sensor_pose->yaw()), mrpt::RAD2DEG(sensor_pose->pitch()),
                 mrpt::RAD2DEG(sensor_pose->roll())});
    }
}