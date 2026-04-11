/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

namespace mola
{
class LoopClosureInterface : public mrpt::rtti::CObject, public mrpt::system::COutputLogger
{
    DEFINE_VIRTUAL_MRPT_OBJECT(LoopClosureInterface, mola)

   public:
    LoopClosureInterface();
    virtual ~LoopClosureInterface();

    // Disable copy and move operations
    LoopClosureInterface(const LoopClosureInterface&)            = delete;
    LoopClosureInterface& operator=(const LoopClosureInterface&) = delete;
    LoopClosureInterface(LoopClosureInterface&&)                 = delete;
    LoopClosureInterface& operator=(LoopClosureInterface&&)      = delete;

    /** @name Main API
     * @{ */

    virtual void initialize(const mrpt::containers::yaml& cfg) = 0;

    /** Find and apply loop closures in the input/output simplemap */
    virtual void process(mrpt::maps::CSimpleMap& sm) = 0;
};

}  // namespace mola