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
 * @file   Parameters.cpp
 * @brief  Parameters for IMU preintegration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */

#include <mola_state_estimation_simple/Parameters.h>

namespace mola::state_estimation_simple
{

void Parameters::loadFrom(const mrpt::containers::yaml& cfg)
{
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);

    MCP_LOAD_OPT(cfg, sigma_relative_pose_linear);
    MCP_LOAD_OPT(cfg, sigma_relative_pose_angular);

    MCP_LOAD_OPT(cfg, sigma_imu_angular_velocity);

    MCP_LOAD_OPT(cfg, enforce_planar_motion);

    MCP_LOAD_OPT(cfg, do_process_imu_labels_re);
    MCP_LOAD_OPT(cfg, do_process_odometry_labels_re);
    MCP_LOAD_OPT(cfg, do_process_gnss_labels_re);

    if (cfg.has("initial_twist"))
    {
        ASSERT_(cfg["initial_twist"].isSequence() && cfg["initial_twist"].asSequence().size() == 6);

        auto&      tw  = initial_twist;
        const auto seq = cfg["initial_twist"].asSequenceRange();
        for (size_t i = 0; i < 6; i++)
        {
            tw[i] = seq.at(i).as<double>();
        }
    }
}

}  // namespace mola::state_estimation_simple
