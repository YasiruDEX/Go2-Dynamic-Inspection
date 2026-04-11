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
 * @file   ImuTransformer.cpp
 * @brief  Transforms IMU readings from an arbitrary sensor pose to the vehicle reference base_link
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */

#include <mola_imu_preintegration/ImuTransformer.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/obs/CObservationIMU.h>

using namespace mola::imu;

mrpt::obs::CObservationIMU ImuTransformer::process(const mrpt::obs::CObservationIMU& raw_imu)
{
    mrpt::obs::CObservationIMU imu = raw_imu;

    const bool hasAngVel = raw_imu.has(mrpt::obs::IMU_WX) && raw_imu.has(mrpt::obs::IMU_WY) &&
                           raw_imu.has(mrpt::obs::IMU_WZ);

    mrpt::math::TVector3D ang_vel_body = {0, 0, 0};
    mrpt::math::TVector3D ang_acc      = {0, 0, 0};

    if (hasAngVel)
    {
        // Transform angular velocity:
        const auto ang_vel_sensor = mrpt::math::TVector3D(  //
            raw_imu.get(mrpt::obs::IMU_WX),  //
            raw_imu.get(mrpt::obs::IMU_WY),  //
            raw_imu.get(mrpt::obs::IMU_WZ));

        ang_vel_body = imu.sensorPose.rotateVector(ang_vel_sensor);
        imu.set(mrpt::obs::IMU_WX, ang_vel_body.x);
        imu.set(mrpt::obs::IMU_WY, ang_vel_body.y);
        imu.set(mrpt::obs::IMU_WZ, ang_vel_body.z);

        // Estimate angular acceleration:
        const auto this_stamp = mrpt::Clock::toDouble(raw_imu.timestamp);
        double     dt         = this_stamp - last_stamp_;
        if (dt <= 0 || dt > 1.0)
        {
            // It's either the first reading, an error, or data flow stopped and resumed.
            // Then use default rate:
            dt = 1.0 / 100.0;
        }

        ang_acc = (ang_vel_body - last_ang_vel_body_) / dt;

        last_ang_vel_body_ = ang_vel_body;
        last_stamp_        = this_stamp;
    }

    // Transform acceleration:
    if (raw_imu.has(mrpt::obs::IMU_X_ACC) && raw_imu.has(mrpt::obs::IMU_Y_ACC) &&
        raw_imu.has(mrpt::obs::IMU_Z_ACC))
    {
        const auto raw_accel_sensor = mrpt::math::TVector3D(  //
            raw_imu.get(mrpt::obs::IMU_X_ACC),  //
            raw_imu.get(mrpt::obs::IMU_Y_ACC),  //
            raw_imu.get(mrpt::obs::IMU_Z_ACC));

        // Rotate acceleration from sensor frame to body frame:
        const auto accel_body_rotated = imu.sensorPose.rotateVector(raw_accel_sensor);

        //  a_imu  = a_body + α×t + ω×(ω×t)  ==>
        //  a_body = R * a_imu - α×t - ω×(ω×t)
        const auto t = raw_imu.sensorPose.translation();

        const auto accel_body =
            accel_body_rotated - mrpt::math::crossProduct3D(ang_acc, t) -
            mrpt::math::crossProduct3D(ang_vel_body, mrpt::math::crossProduct3D(ang_vel_body, t));

        imu.set(mrpt::obs::IMU_X_ACC, accel_body.x);
        imu.set(mrpt::obs::IMU_Y_ACC, accel_body.y);
        imu.set(mrpt::obs::IMU_Z_ACC, accel_body.z);
    }

    // Mark the new reference frame:
    imu.sensorPose = mrpt::poses::CPose3D::Identity();

    return imu;
}
