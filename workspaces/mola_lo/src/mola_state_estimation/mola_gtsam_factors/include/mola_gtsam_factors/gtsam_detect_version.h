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

#pragma once

#include <gtsam/config.h>

#define GTSAM_VERSION_AT_LEAST(major, minor, patch)                       \
    ((GTSAM_VERSION_MAJOR > (major)) ||                                   \
     (GTSAM_VERSION_MAJOR == (major) && GTSAM_VERSION_MINOR > (minor)) || \
     (GTSAM_VERSION_MAJOR == (major) && GTSAM_VERSION_MINOR == (minor) && \
      GTSAM_VERSION_PATCH >= (patch)))

#define GTSAM_USES_BOOST (!GTSAM_VERSION_AT_LEAST(4, 3, 0))
