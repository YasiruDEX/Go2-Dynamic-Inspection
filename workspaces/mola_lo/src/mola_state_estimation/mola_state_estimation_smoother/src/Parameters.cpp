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
 * @brief  Parameters for NavStateFuse
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_state_estimation_smoother/Parameters.h>

namespace mola::state_estimation_smoother
{

void Parameters::loadFrom(const mrpt::containers::yaml& cfg)
{
    // Reference frame IDs
    // -----------------------------------------------------
    MCP_LOAD_REQ(cfg, vehicle_frame_name);
    MCP_LOAD_REQ(cfg, reference_frame_name);
    MCP_LOAD_OPT(cfg, enu_frame_name);

    // Kinematic factors and keyframe creation (motion model)
    // -------------------------------------------------------
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_REQ(cfg, sliding_window_length);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);
    MCP_LOAD_OPT(cfg, sigma_integrator_position);
    MCP_LOAD_OPT(cfg, sigma_integrator_orientation);
    MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_linear);
    MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_angular);

    MCP_LOAD_OPT(cfg, min_time_difference_to_create_new_frame);
    MCP_LOAD_OPT(cfg, time_between_frames_to_warning);
    MCP_LOAD_OPT(cfg, gnss_nearby_keyframe_stamp_tolerance);
    MCP_LOAD_OPT(cfg, imu_nearby_keyframe_stamp_tolerance);

    MCP_LOAD_OPT(cfg, initial_twist_sigma_lin);
    MCP_LOAD_OPT(cfg, initial_twist_sigma_ang);

    MCP_LOAD_OPT(cfg, enforce_planar_motion);

    if (cfg.has("link_first_pose_to_reference_origin_sigma"))
    {
        const auto strSigma = cfg["link_first_pose_to_reference_origin_sigma"].as<std::string>();
        double     sigma    = 0;
        if (1 == ::sscanf(strSigma.c_str(), "%lf", &sigma))
        {
            ASSERT_GT_(sigma, .0);
            link_first_pose_to_reference_origin_sigma = sigma;
        }
    }

    // IMU-related
    // -----------------------------------------------------
    MCP_LOAD_OPT(cfg, imu_attitude_sigma_deg);
    MCP_LOAD_OPT(cfg, imu_attitude_azimuth_offset_deg);
    MCP_LOAD_OPT(cfg, imu_normalized_gravity_alignment_sigma);

    // Geo-referencing
    // -----------------------------------------------------
    MCP_LOAD_OPT(cfg, estimate_geo_reference);

    if (cfg.has("fixed_geo_reference"))
    {
        auto& gr = fixed_geo_reference.emplace();

        const auto& fgr = cfg["fixed_geo_reference"];

        ASSERT_(fgr.isMap());
        ASSERT_(fgr.has("latitude_deg"));
        ASSERT_(fgr.has("longitude_deg"));
        ASSERT_(fgr.has("altitude"));

        gr.geo_coord.lat    = fgr["latitude_deg"].as<double>();
        gr.geo_coord.lon    = fgr["longitude_deg"].as<double>();
        gr.geo_coord.height = fgr["altitude"].as<double>();

        // TODO: Allow defining a custom transformation != Identity?
        gr.T_enu_to_map.cov.setIdentity();
        gr.T_enu_to_map.cov *= 1e-6;
    }

    MCP_LOAD_OPT(cfg, convergence_max_position_sigma);
    MCP_LOAD_OPT(cfg, convergence_max_orientation_sigma_deg);
    MCP_LOAD_OPT(cfg, publish_estimated_georef_on_convergence);

    MCP_LOAD_REQ(cfg, kinematic_model);

    // Nonlinear optimization
    // -----------------------------------------------------
    MCP_LOAD_OPT(cfg, additional_isam2_update_steps);

    // name Sensor input names
    // -----------------------------------------------------
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

    if (cfg.has("visualization"))
    {
        visualization.loadFrom(cfg["visualization"]);
    }
}

void Parameters::Visualization::loadFrom(const mrpt::containers::yaml& cfg)
{
    MCP_LOAD_OPT(cfg, show_console_messages);
}

}  // namespace mola::state_estimation_smoother
