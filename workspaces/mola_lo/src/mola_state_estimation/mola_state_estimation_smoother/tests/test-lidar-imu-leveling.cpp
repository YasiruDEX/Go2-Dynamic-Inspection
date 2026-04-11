/* _
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
 * @file   test-lidar-imu-leveling.cpp
 * @brief  Unit test for fusing LiDAR odometry with IMU (accelerometer)
 * to recover horizontality from a bad initial roll/pitch.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 10, 2025
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mola_state_estimation_smoother/pose_pdf_to_string_with_sigmas.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
const size_t numSteps = 100;
const double T        = 0.05;  // 20 Hz

auto& rng = mrpt::random::getRandomGenerator();

const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"

    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0

    max_time_to_use_velocity_model: 2.0
    
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.5  # Allow quick adaptation

    imu_normalized_gravity_alignment_sigma: 0.1

    # Weak link between "map" frame origin and "odom"
    link_first_pose_to_reference_origin_sigma: 0.01

)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    // 1. Initialize State with a huge Pitch/Roll Error
    // Real robot is flat (Roll=0, Pitch=0), but we tell the filter it's tilted.

    const double initBadPitch = 3.0_deg;
    const double initBadRoll  = 2.0_deg;

    const auto initialBadPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0.0_deg, initBadPitch, initBadRoll);

    // Accumulated Odometry (Simulate typical LiDAR drift)
    mrpt::poses::CPose3D currentOdom = initialBadPose;

    for (size_t i = 1; i <= numSteps; i++)
    {
        const double t    = T * static_cast<double>(i);
        const auto   time = mrpt::Clock::fromDouble(t);

        // --- Generate LiDAR Odometry ---
        // Robot is moving forward on flat ground.
        // Odometry drifts slightly in Z/Pitch/Roll even though real motion is flat.
        auto deltaOdom = mrpt::poses::CPose3D(
            0.5 * T, 0, 0, 0.1 * T, 0, 0  // Turning
        );

        // Add nasty drift to Odometry that would indicate sinking/tilting
        deltaOdom.z_incr(-0.002);  // Drifting down
        deltaOdom.setYawPitchRoll(deltaOdom.yaw(), 0.1_deg /* Pitch drift*/, deltaOdom.roll());

        currentOdom = currentOdom + deltaOdom;

        mrpt::poses::CPose3DPDFGaussian odomLidarPdf;
        odomLidarPdf.mean = currentOdom;
        odomLidarPdf.cov.setIdentity();
        odomLidarPdf.cov *= 1e-3;

        // --- Generate IMU (Accel Only) ---
        // Robot is physically flat, so gravity is straight down in sensor frame (0,0,g).
        mrpt::obs::CObservationIMU obsImu;
        obsImu.timestamp   = time;
        obsImu.sensorLabel = "imu";

        // Pure gravity + noise. No linear acceleration (constant velocity).
        // If the robot were actually tilted 20 deg, the IMU would read gravity components in X/Y.
        // Since the robot IS flat, IMU reads ~0 in X/Y.
        obsImu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0.0, 0.1));
        obsImu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0.0, 0.1));
        obsImu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0.0, 0.1));

        // Send to estimator
        stateEst.fuse_pose(time, odomLidarPdf, "lidar");
        stateEst.fuse_imu(obsImu);
    }

    // Recover final state
    const double last_t   = T * static_cast<double>(numSteps);
    auto         stateOpt = stateEst.estimated_navstate(mrpt::Clock::fromDouble(last_t), "map");

    ASSERT_(stateOpt.has_value());

    double y, p, r;
    stateOpt->pose.mean.getYawPitchRoll(y, p, r);

    std::cout << "Final Estimated Pitch: " << mrpt::RAD2DEG(p) << " deg\n";
    std::cout << "Final Estimated Roll:  " << mrpt::RAD2DEG(r) << " deg\n";

    // Check leveling
    // Tolerances depend on IMU noise and filter tuning, but should be close to 0
    ASSERT_NEAR_(p, 0.0, mrpt::DEG2RAD(4.0));
    ASSERT_NEAR_(r, 0.0, mrpt::DEG2RAD(4.0));

    ASSERT_(stateEst.known_odometry_frame_ids().size() == 1);
    for (const auto& odom_frame : stateEst.known_odometry_frame_ids())
    {
        auto T_map_to_odom = stateEst.estimated_T_map_to_odometry_frame(odom_frame);
        ASSERT_(T_map_to_odom.has_value());
        std::cout << "T_map_to_odom[" << odom_frame << "]:\n"
                  << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(*T_map_to_odom)
                  << "\n";
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        run_test();
        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}