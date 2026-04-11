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
 * @file   FactorGnssMap2Enu.cpp
 * @brief  GNSS factor for usage with a T_enu2_map transform symbol
 * @author Jose Luis Blanco Claraco
 * @date   Dec 3, 2025
 */

#include <mola_gtsam_factors/FactorGnssMapEnu.h>

namespace mola::factors
{

FactorGnssMapEnu::FactorGnssMapEnu() = default;

}