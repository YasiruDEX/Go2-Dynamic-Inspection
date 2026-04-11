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
 * @file   test-state-estimation-simple.cpp
 * @brief  Unit tests for StateEstimationSimple
 * @author Jose Luis Blanco Claraco
 * @date   Dec 10, 2025
 */

#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// Helper to create a configuration
mrpt::containers::yaml get_default_config()
{
    const char* yaml_text = R"###(
params:
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_relative_pose_linear: 0.1
    sigma_relative_pose_angular: 0.1
    sigma_imu_angular_velocity: 0.05
    enforce_planar_motion: false
)###";
    return mrpt::containers::yaml::FromText(yaml_text);
}

// --------------------------------------------------------------------------
// Test 1: Pose Fusion & Twist Calculation
// --------------------------------------------------------------------------
// Feed two poses. The estimator should calculate the velocity (twist) between
// them and be able to extrapolate to a future time.
void test_pose_and_twist()
{
    std::cout << "[Test 1] Pose Fusion & Twist Calculation... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    const double v_x = 2.0;  // m/s

    // t=0: Initial Pose at origin
    {
        auto pdf = mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity());
        estimator.fuse_pose(mrpt::Clock::fromDouble(0.0), pdf, "map");
    }

    // t=1: Moved 2 meters in X
    {
        auto pdf = mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D(v_x, 0, 0), mrpt::math::CMatrixDouble66::Identity());
        estimator.fuse_pose(mrpt::Clock::fromDouble(1.0), pdf, "map");
    }

    // Check 1: Twist should have been calculated
    auto twistOpt = estimator.get_last_twist();
    ASSERT_(twistOpt.has_value());
    ASSERT_NEAR_(twistOpt->vx, v_x, 1e-3);
    ASSERT_NEAR_(twistOpt->vy, 0.0, 1e-3);

    // Check 2: Extrapolation to t=1.5
    // Should be at x = 2.0 + (0.5 * 2.0) = 3.0
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.5), "map");
    ASSERT_(stateOpt.has_value());
    ASSERT_NEAR_(stateOpt->pose.mean.x(), 3.0, 1e-3);

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 2: Odometry Fusion
// --------------------------------------------------------------------------
// Initialize, then feed differential odometry.
void test_odometry_fusion()
{
    std::cout << "[Test 2] Odometry Fusion... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    // 1. Must initialize with a known pose
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // 2. Initialize Odometry baseline (t=0)
    {
        mrpt::obs::CObservationOdometry odom;
        odom.timestamp = mrpt::Clock::fromDouble(0.0);
        odom.odometry  = mrpt::poses::CPose2D::Identity();
        estimator.fuse_odometry(odom);
    }

    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.1),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // 3. Feed Odometry increment (t=1)
    // Robot moved 1m in X according to wheel encoders
    {
        mrpt::obs::CObservationOdometry odom;
        odom.timestamp = mrpt::Clock::fromDouble(1.0);
        odom.odometry  = mrpt::poses::CPose2D(1.0, 0, 0);  // 1m forward

        // Could be: estimator.fuse_odometry(odom);
        // But let's test the RawDataConsumer API:
        estimator.onNewObservation(std::make_shared<const mrpt::obs::CObservationOdometry>(odom));
    }

    // Check: Current state should reflect the odometry increment
    // Note: StateEstimationSimple logic sums the increment to the last pose.
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.0), "map");
    ASSERT_(stateOpt.has_value());
    ASSERT_NEAR_(stateOpt->pose.mean.x(), 1.0, 1e-3);

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 3: IMU Fusion (Angular Velocity)
// --------------------------------------------------------------------------
// The simple estimator uses IMU to overwrite the angular velocity (twist)
// for extrapolation.
void test_imu_angular_velocity()
{
    std::cout << "[Test 3] IMU Angular Velocity... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    // Initialize at origin
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // Force a twist via IMU at t=0.1
    // Rotating around Z at 1.0 rad/s
    {
        mrpt::obs::CObservationIMU imu;
        imu.timestamp = mrpt::Clock::fromDouble(0.1);
        imu.set(mrpt::obs::IMU_WX, 0.0);
        imu.set(mrpt::obs::IMU_WY, 0.0);
        imu.set(mrpt::obs::IMU_WZ, 1.0);  // 1 rad/s
        imu.sensorPose = mrpt::poses::CPose3D::Identity();  // IMU aligned with body

        // Could be also: estimator.fuse_imu(imu);
        estimator.onNewObservation(std::make_shared<const mrpt::obs::CObservationIMU>(imu));
    }

    // Check if twist was updated
    auto twist = estimator.get_last_twist();
    ASSERT_(twist.has_value());
    ASSERT_NEAR_(twist->wz, 1.0, 1e-3);

    // Predict state at t=1.1 (delta = 1.0s from last *twist* update?
    // Wait, estimated_navstate calculates dt from last_pose_obs_tim (t=0.0)
    // So dt = 1.1s.
    // Rotation = w * dt = 1.0 * 1.1 = 1.1 rad.
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.1), "map");

    ASSERT_(stateOpt.has_value());

    double y, p, r;
    stateOpt->pose.mean.getYawPitchRoll(y, p, r);

    // Logic check: The class uses the *last stored twist* to extrapolate from *last stored pose*.
    // Last pose t=0.0. Last twist is the one set by IMU. Target t=1.1. dt=1.1.
    // Expected yaw = 1.1 rad.
    ASSERT_NEAR_(y, 1.1, 1e-3);

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 4: Planar Motion Enforcement
// --------------------------------------------------------------------------
// Verify that Z, Pitch, and Roll are zeroed out if config requests it.
void test_planar_motion()
{
    std::cout << "[Test 4] Planar Motion Constraint... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    auto                                                 cfg = get_default_config();
    cfg["params"]["enforce_planar_motion"]                   = true;
    estimator.initialize(cfg);

    // Feed a non-planar pose (Z height, Roll, Pitch)
    auto nonPlanarPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        1.0, 2.0, 5.0,  // X, Y, Z=5
        1.0, 0.5, 0.2);  // Yaw, Pitch=0.5, Roll=0.2

    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(nonPlanarPose, mrpt::math::CMatrixDouble66::Identity()),
        "map");
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.05),
        mrpt::poses::CPose3DPDFGaussian(nonPlanarPose, mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // Check immediate estimate
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(0.1), "map");

    ASSERT_(stateOpt.has_value());

    // Z should be 0, Pitch 0, Roll 0. X/Y/Yaw should remain (approximately).
    const auto& p = stateOpt->pose.mean;
    ASSERT_NEAR_(p.z(), 0.0, 1e-5);

    double y, pit, rol;
    p.getYawPitchRoll(y, pit, rol);
    ASSERT_NEAR_(pit, 0.0, 1e-5);
    ASSERT_NEAR_(rol, 0.0, 1e-5);
    ASSERT_NEAR_(y, 1.0, 1e-5);  // Yaw should be preserved

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 5: GNSS Ignored
// --------------------------------------------------------------------------
// Verify that GNSS messages don't crash the system (explicitly ignored in source).
void test_gnss_ignored()
{
    std::cout << "[Test 5] GNSS Ignored... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.initialize(get_default_config());

    // Init
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.05),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // Feed GNSS
    mrpt::obs::CObservationGPS gps;
    gps.timestamp = mrpt::Clock::fromDouble(0.5);
    // Just ensure it doesn't throw
    try
    {
        estimator.fuse_gnss(gps);
    }
    catch (...)
    {
        std::cerr << "fuse_gnss threw an exception!\n";
        throw;
    }

    // State should remain unchanged/valid
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(0.0), "map");
    ASSERT_(stateOpt.has_value());

    std::cout << "OK\n";
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_pose_and_twist();
        test_odometry_fusion();
        test_imu_angular_velocity();
        test_planar_motion();
        test_gnss_ignored();

        std::cout << "\nAll StateEstimationSimple tests passed!\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\nERROR: " << e.what() << std::endl;
        return 1;
    }
}