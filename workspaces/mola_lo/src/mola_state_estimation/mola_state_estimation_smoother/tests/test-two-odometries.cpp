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
 * @file   test-two-odometries.cpp
 * @brief  Unit tests for fusing two sources of odometry (Wheels + LiDAR)
 * with drift and white noise.
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
// Simulation parameters
constexpr double WHEEL_ODOM_NOISE_XY  = 0.05;  // m per step
constexpr double WHEEL_ODOM_NOISE_PHI = 0.02;  // rad per step
constexpr double WHEEL_ODOM_DRIFT_X   = 1.02;  // Systematic scale error (2% drift)

constexpr double LIDAR_ODOM_NOISE_XYZ = 0.01;  // More accurate than wheels
constexpr double LIDAR_ODOM_NOISE_ANG = 0.001;
constexpr double LIDAR_ODOM_DRIFT_Y   = 0.005;  // Constant drift in Y per step

const bool   VERBOSE          = mrpt::get_env<bool>("VERBOSE", false);
const size_t numPoses         = 500;
const size_t EVALUATE_EVERY_N = 10;
const double T                = 0.1;

auto& rng = mrpt::random::getRandomGenerator();

const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"

    # Link the "map" frame origin with "odom" for this toy example.
    # Otherwise, "odom" would "float" around without any particular XYZ known displacement.
    link_first_pose_to_reference_origin_sigma: 1e-6

    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0

    max_time_to_use_velocity_model: 2.0
    
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10

    estimate_geo_reference: false
)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;

    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }

    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    // Ground Truth State
    mrpt::poses::CPose3D gtPose = mrpt::poses::CPose3D::Identity();

    // To evaluate error as increments:
    mrpt::poses::CPose3D gtLastCheckPose  = gtPose;
    mrpt::poses::CPose3D estLastCheckPose = mrpt::poses::CPose3D::Identity();

    // Accumulated Drifting Odom States
    mrpt::poses::CPose3D odomWheels =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(-20.0, 10.0, 0.0, 42.0_deg, 0.0_deg, 0.0_deg);
    auto odomWheelsLastCheckPose = odomWheels;

    mrpt::poses::CPose3D odomLidar              = mrpt::poses::CPose3D::Identity();
    auto                 odomLidarLastCheckPose = odomLidar;

    // Constant ground truth velocity (1 m/s X, 0.2 rad/s Yaw)
    const auto gtDelta = mrpt::poses::CPose3D(1.0 * T, 0, 0, 0.2 * T, 0, 0);

    for (size_t i = 0; i < numPoses; i++)
    {
        const double t     = T * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);

        // 1. Update Ground Truth
        if (i > 0)
        {
            gtPose = gtPose + gtDelta;
        }

        // 2. Generate Wheel Odometry (Noisier, scale drift)
        // Simulate systematic scale drift in X
        auto deltaWheels = gtDelta;
        deltaWheels.x(deltaWheels.x() * WHEEL_ODOM_DRIFT_X);

        // Add White Noise
        deltaWheels.x_incr(rng.drawGaussian1D(0, WHEEL_ODOM_NOISE_XY));
        deltaWheels.y_incr(rng.drawGaussian1D(0, WHEEL_ODOM_NOISE_XY));
        deltaWheels.setYawPitchRoll(
            deltaWheels.yaw() + rng.drawGaussian1D(0, WHEEL_ODOM_NOISE_PHI), deltaWheels.pitch(),
            deltaWheels.roll());

        odomWheels = odomWheels + deltaWheels;

        // High covariance for wheels (from observation motion model params)
        mrpt::poses::CPose3DPDFGaussian odomWheelsPdf;
        odomWheelsPdf.mean = odomWheels;
        odomWheelsPdf.cov.setIdentity();
        for (int k = 0; k < 3; k++)
        {
            odomWheelsPdf.cov(k, k) = mrpt::square(WHEEL_ODOM_NOISE_XY);
            odomWheelsPdf.cov(k, k) = mrpt::square(WHEEL_ODOM_NOISE_PHI);
        }

        // 3. Generate LiDAR Odometry (Less noise, sideways drift)
        auto deltaLidar = gtDelta;
        // Systematic drift in Y
        deltaLidar.y_incr(LIDAR_ODOM_DRIFT_Y);

        // Add White Noise
        deltaLidar.x_incr(rng.drawGaussian1D(0, LIDAR_ODOM_NOISE_XYZ));
        deltaLidar.y_incr(rng.drawGaussian1D(0, LIDAR_ODOM_NOISE_XYZ));
        deltaLidar.z_incr(rng.drawGaussian1D(0, LIDAR_ODOM_NOISE_XYZ));
        deltaLidar.setYawPitchRoll(
            deltaLidar.yaw() + rng.drawGaussian1D(0, LIDAR_ODOM_NOISE_ANG),
            deltaLidar.pitch() + rng.drawGaussian1D(0, LIDAR_ODOM_NOISE_ANG),
            deltaLidar.roll() + rng.drawGaussian1D(0, LIDAR_ODOM_NOISE_ANG));

        odomLidar = odomLidar + deltaLidar;

        // Low covariance for lidar
        mrpt::poses::CPose3DPDFGaussian odomLidarPdf;
        odomLidarPdf.mean = odomLidar;
        odomLidarPdf.cov.setIdentity();
        for (int k = 0; k < 3; k++)
        {
            odomLidarPdf.cov(k, k) = mrpt::square(LIDAR_ODOM_NOISE_XYZ);
            odomLidarPdf.cov(k, k) = mrpt::square(LIDAR_ODOM_NOISE_ANG);
        }

        // 4. Fuse Both
        stateEst.fuse_pose(stamp, odomWheelsPdf, "wheels_odom");
        stateEst.fuse_pose(stamp, odomLidarPdf, "lidar_odom");

        if ((i % EVALUATE_EVERY_N) == 0)
        {
            // Evaluate every K steps:
            auto stateOpt =
                stateEst.estimated_navstate(stamp, stateEst.parameters().reference_frame_name);

            ASSERT_(stateOpt.has_value());
            const auto estPose = stateOpt->pose.mean;

            const auto gtPoseIncrement     = gtPose - gtLastCheckPose;
            const auto estPoseIncrement    = estPose - estLastCheckPose;
            const auto lidarPoseIncrement  = odomLidar - odomLidarLastCheckPose;
            const auto wheelsPoseIncrement = odomWheels - odomWheelsLastCheckPose;

            gtLastCheckPose         = gtPose;
            estLastCheckPose        = estPose;
            odomLidarLastCheckPose  = odomLidar;
            odomWheelsLastCheckPose = odomWheels;

            std::cout << "t=" << t << "\n";
            std::cout << "  GT Pose:    " << gtPose.asString() << "\n";
            std::cout << "  Wheel Odom: " << odomWheels.asString() << "\n";
            std::cout << "  Lidar Odom: " << odomLidar.asString() << "\n";
            std::cout << "  Fused Est:  " << estPose.asString() << "\n";
            std::cout << " Deltas:\n";
            std::cout << "  GT DeltaPose    : " << gtPoseIncrement.asString() << "\n";
            std::cout << "  Est DeltaPose   : " << estPoseIncrement.asString() << "\n";
            std::cout << "  LO DeltaPose    : " << lidarPoseIncrement.asString() << "\n";
            std::cout << "  WheelsDeltaPose : " << wheelsPoseIncrement.asString() << "\n";

#if 0
            const double err_wheels_delta =
                mrpt::poses::Lie::SE<3>::log(wheelsPoseIncrement - gtPoseIncrement).norm();
            const double err_lidar_delta =
                mrpt::poses::Lie::SE<3>::log(lidarPoseIncrement - gtPoseIncrement).norm();
            const double err_fused_delta =
                mrpt::poses::Lie::SE<3>::log(estPoseIncrement - gtPoseIncrement).norm();
#endif
            const double err_wheels = mrpt::poses::Lie::SE<3>::log(odomWheels - gtPose).norm();
            const double err_lidar  = mrpt::poses::Lie::SE<3>::log(odomLidar - gtPose).norm();
            const double err_fused  = mrpt::poses::Lie::SE<3>::log(estPose - gtPose).norm();

            std::cout << "  Error Wheels: " << err_wheels << "\n";
            std::cout << "  Error Lidar:  " << err_lidar << "\n";
            std::cout << "  Error Fused:  " << err_fused << "\n";

            // The fused error should be reasonable.
            ASSERT_LT_(err_fused, err_wheels);
        }
    }

    ASSERT_(stateEst.known_odometry_frame_ids().size() == 2);
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