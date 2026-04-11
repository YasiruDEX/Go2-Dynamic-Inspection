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
 * @file   ImuTransformer.h
 * @brief  Transforms IMU readings from an arbitrary sensor pose to the vehicle reference base_link
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/obs_frwds.h>

namespace mola::imu
{
/** Transforms IMU readings from an arbitrary sensor pose to the vehicle reference "base_link".
 *
 * This basically rotates the angular velocity vector according to `raw_imu.sensorPose` and
 * transforms the acceleration vector according to the lever between vehicle "base_link" and the
 * sensor pose.
 * The returned IMU reading object has `sensorPose` set to the SE(3) identity to reflect its new
 * frame.
 *
 * Note that bias is *not* subtracted here, it must be done by the caller.
 * Also, gravity is not subtracted here since this class has no knowledge of global orientation.
 *
 * Since this class has a state (the stamp of the last observation), one object must be instantiated
 * for each IMU sensor.
 *
 * \ingroup mola_imu_preintegration_grp
 */
class ImuTransformer
{
   public:
    ImuTransformer() = default;

    mrpt::obs::CObservationIMU process(const mrpt::obs::CObservationIMU& raw_imu);

   private:
    double                last_stamp_ = 0;
    mrpt::math::TVector3D last_ang_vel_body_;
};

}  // namespace mola::imu
