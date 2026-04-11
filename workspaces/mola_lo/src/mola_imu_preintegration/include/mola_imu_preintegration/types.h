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

#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

namespace mola::imu
{

using TimeStamp           = double;  //!< seconds in UNIX epoch
using SO3                 = mrpt::math::CMatrixDouble33;
using LinearVelocity      = mrpt::math::TVector3D;
using LinearAcceleration  = mrpt::math::TVector3D;
using AngularAcceleration = mrpt::math::TVector3D;
using AngularVelocity     = mrpt::math::TVector3D;

}  // namespace mola::imu
