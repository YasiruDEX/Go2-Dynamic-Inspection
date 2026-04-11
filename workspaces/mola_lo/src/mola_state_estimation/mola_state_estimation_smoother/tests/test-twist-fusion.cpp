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
 * @file   test-twist-fusion.cpp
 * @brief  Unit tests for StateEstimationSmoother twist fusion
 * @author Jose Luis Blanco Claraco
 * @date   Dec 10, 2025
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mola_state_estimation_smoother/pose_pdf_to_string_with_sigmas.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{

constexpr double TWIST_NOISE_LINEAR  = 0.05;  // m/s
constexpr double TWIST_NOISE_ANGULAR = 0.02;  // rad/s

constexpr double MAXIMUM_POSITION_ERROR = 0.30;  // meters
constexpr double MAXIMUM_VELOCITY_ERROR = 0.20;  // m/s

constexpr const char* ODOMETRY_NAME = "odom";

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

const size_t numSteps = 50;
const double T        = 0.1;  // time step (10 Hz)

auto& rng = mrpt::random::getRandomGenerator();

const char* navStateParams =
    R"###(# Config for Parameters
params:
    # Frame name for the vehicle/robot base
    vehicle_frame_name: "base_link"

    # Reference frame for pose publication (typically 'map' or 'odom')
    reference_frame_name: "map"

    max_time_to_use_velocity_model: 2.0

    # ----------------------------------------------------------------------------
    # Kinematic Model & Motion Factors
    # ----------------------------------------------------------------------------

    # Kinematic model for internal motion model factors
    kinematic_model: KinematicModel::ConstantVelocity

    # Time window to keep past observations in the filter [seconds]
    sliding_window_length: 3.0
    
    # Minimum time difference between frames to create a new frame [seconds]
    min_time_difference_to_create_new_frame: 0.05

    # Random walk model for linear acceleration uncertainty [m/s²]
    sigma_random_walk_acceleration_linear: 1.0

    # Random walk model for angular acceleration uncertainty [rad/s²]
    sigma_random_walk_acceleration_angular: 1.0

    # Integrator uncertainty for position [m]
    sigma_integrator_position: 0.10

    # Integrator uncertainty for orientation [rad]
    sigma_integrator_orientation: 0.10

    # Twist from consecutive poses uncertainty
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0

    estimate_geo_reference: false
)###";

using Pose   = mrpt::poses::CPose3D;
using Twist  = mrpt::math::TTwist3D;
using Twist2 = mrpt::math::TTwist2D;

struct TestCase
{
    Pose        initial_pose;
    Twist       constant_twist;  // Ground truth constant twist
    std::string description;
};

// Test: Simulate a vehicle moving with constant twist (linear and angular velocities).
// Fuse twist observations and verify the estimator can recover the trajectory.
//
void run_test(const TestCase& testCase)
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;

    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.profiler_.enable();

    {
        auto cfgYaml = mrpt::containers::yaml::FromText(navStateParams);
        stateEst.initialize(cfgYaml);
    }

    const auto gtInitialPose = testCase.initial_pose;
    const auto gtTwist       = testCase.constant_twist;

    // Current ground truth pose
    mrpt::poses::CPose3D currentGtPose = gtInitialPose;

    // Link the "map" frame with "odom" for this example
    {
        auto map2odom_cov = mrpt::math::CMatrixDouble66::Identity();
        map2odom_cov *= 1e-6;

        stateEst.fuse_pose(
            mrpt::Clock::fromDouble(0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), map2odom_cov),
            stateEst.parameters().reference_frame_name);
    }

    if (VERBOSE)
    {
        std::cout << "\n=== Test Configuration ===\n";
        std::cout << "Description: " << testCase.description << "\n";
        std::cout << "Initial pose: " << gtInitialPose << "\n";
        std::cout << "Constant twist: vx=" << gtTwist.vx << " vy=" << gtTwist.vy
                  << " vz=" << gtTwist.vz << " wx=" << gtTwist.wx << " wy=" << gtTwist.wy
                  << " wz=" << gtTwist.wz << "\n\n";
    }

    // Apply rotation (exponential map for angular velocity)
    const auto deltaPose = [&]()
    {
        mrpt::math::CVectorFixedDouble<6> delta;
        delta[0] = gtTwist.vx * T;
        delta[1] = gtTwist.vy * T;
        delta[2] = gtTwist.vz * T;
        delta[3] = gtTwist.wx * T;
        delta[4] = gtTwist.wy * T;
        delta[5] = gtTwist.wz * T;

        return mrpt::poses::Lie::SE<3>::exp(delta);
    }();

    for (size_t i = 0; i <= numSteps; i++)
    {
        const double t = T * static_cast<double>(i);

        // 1. Update ground truth pose by integrating twist
        if (i > 0)
        {
            // Simple Euler integration: pose = pose + twist * dt

            currentGtPose = currentGtPose + deltaPose;
        }

        // 2. Simulate noisy twist observation
        {
            mrpt::math::TTwist3D noisyTwist;
            noisyTwist.vx = gtTwist.vx + rng.drawGaussian1D(0, TWIST_NOISE_LINEAR);
            noisyTwist.vy = gtTwist.vy + rng.drawGaussian1D(0, TWIST_NOISE_LINEAR);
            noisyTwist.vz = gtTwist.vz + rng.drawGaussian1D(0, TWIST_NOISE_LINEAR);
            noisyTwist.wx = gtTwist.wx + rng.drawGaussian1D(0, TWIST_NOISE_ANGULAR);
            noisyTwist.wy = gtTwist.wy + rng.drawGaussian1D(0, TWIST_NOISE_ANGULAR);
            noisyTwist.wz = gtTwist.wz + rng.drawGaussian1D(0, TWIST_NOISE_ANGULAR);

            // Twist covariance matrix (6x6)
            mrpt::math::CMatrixDouble66 twistCov;
            twistCov.setIdentity();
            twistCov(0, 0) = twistCov(1, 1) = twistCov(2, 2) = mrpt::square(TWIST_NOISE_LINEAR);
            twistCov(3, 3) = twistCov(4, 4) = twistCov(5, 5) = mrpt::square(TWIST_NOISE_ANGULAR);

            // Fuse twist observation
            stateEst.fuse_twist(mrpt::Clock::fromDouble(t), noisyTwist, twistCov);

            if (VERBOSE && i % 10 == 0)
            {
                std::cout << "t=" << t << " Twist: vx=" << noisyTwist.vx << " vy=" << noisyTwist.vy
                          << " wz=" << noisyTwist.wz << "\n";
            }
        }

        // 3. Optionally add some odometry to help with pose estimation
        // (twist alone doesn't provide absolute position)
        if (i % 5 == 0)
        {
            // Add noise to odometry
            auto noisyPose = currentGtPose;
            noisyPose.x_incr(rng.drawGaussian1D(0, 0.05));
            noisyPose.y_incr(rng.drawGaussian1D(0, 0.05));
            noisyPose.z_incr(rng.drawGaussian1D(0, 0.05));

            mrpt::poses::CPose3DPDFGaussian odoPose;
            odoPose.mean = noisyPose;
            odoPose.cov.setDiagonal(mrpt::square(0.05));

            stateEst.fuse_pose(mrpt::Clock::fromDouble(t), odoPose, ODOMETRY_NAME);
        }

        // Check estimation periodically
        if (i > 0 && i % 10 == 0)
        {
            const auto stateOpt = stateEst.estimated_navstate(
                mrpt::Clock::fromDouble(t), stateEst.parameters().reference_frame_name);

            if (stateOpt && VERBOSE)
            {
                const auto estimatedPose  = stateOpt->pose.mean;
                const auto estimatedTwist = stateOpt->twist;

                std::cout << "\n--- Estimation at t=" << t << " ---\n";
                std::cout << "Estimated pose: " << estimatedPose << "\n";
                std::cout << "Ground truth pose: " << currentGtPose << "\n";
                std::cout << "Relative GT pose: " << currentGtPose - gtInitialPose << "\n";
                std::cout << "Estimated twist: vx=" << estimatedTwist.vx
                          << " vy=" << estimatedTwist.vy << " wz=" << estimatedTwist.wz << "\n";
                std::cout << "Ground truth twist: vx=" << gtTwist.vx << " vy=" << gtTwist.vy
                          << " wz=" << gtTwist.wz << "\n";

                const auto posError =
                    (estimatedPose.asTPose() - (currentGtPose - gtInitialPose).asTPose()).norm();
                const auto velError = std::sqrt(
                    mrpt::square(estimatedTwist.vx - gtTwist.vx) +
                    mrpt::square(estimatedTwist.vy - gtTwist.vy) +
                    mrpt::square(estimatedTwist.vz - gtTwist.vz));

                std::cout << "Position error: " << posError << " m\n";
                std::cout << "Velocity error: " << velError << " m/s\n\n";
            }
        }
    }

    for (const auto& odom_frame : stateEst.known_odometry_frame_ids())
    {
        auto T_map_to_odom = stateEst.estimated_T_map_to_odometry_frame(odom_frame);
        ASSERT_(T_map_to_odom.has_value());
        std::cout << "T_map_to_odom[" << odom_frame << "]:\n"
                  << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(*T_map_to_odom)
                  << "\n";
    }

    // Final verification
    const double last_t = T * static_cast<double>(numSteps);
    {
        const auto stateOpt = stateEst.estimated_navstate(
            mrpt::Clock::fromDouble(last_t), stateEst.parameters().reference_frame_name);

        ASSERT_(stateOpt.has_value());

        const auto estimatedPose  = stateOpt->pose.mean;
        const auto estimatedTwist = stateOpt->twist;

        std::cout << "\n=== FINAL RESULTS ===\n";
        std::cout << "Estimated pose: " << estimatedPose << "\n";
        std::cout << "Ground truth pose: " << currentGtPose << "\n";
        std::cout << "Relative GT pose: " << currentGtPose - gtInitialPose << "\n";
        std::cout << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(stateOpt->pose)
                  << "\n";

        // Check position error
        const auto positionError =
            (estimatedPose.asTPose() - (currentGtPose - gtInitialPose).asTPose()).norm();
        std::cout << "Position error: " << positionError << " m\n";
        ASSERT_LT_(positionError, MAXIMUM_POSITION_ERROR);

        // Check velocity error
        const auto velocityError = std::sqrt(
            mrpt::square(estimatedTwist.vx - gtTwist.vx) +
            mrpt::square(estimatedTwist.vy - gtTwist.vy) +
            mrpt::square(estimatedTwist.vz - gtTwist.vz));
        std::cout << "Linear velocity error: " << velocityError << " m/s\n";
        ASSERT_LT_(velocityError, MAXIMUM_VELOCITY_ERROR);

        // Check angular velocity error
        const auto angVelError = std::sqrt(
            mrpt::square(estimatedTwist.wx - gtTwist.wx) +
            mrpt::square(estimatedTwist.wy - gtTwist.wy) +
            mrpt::square(estimatedTwist.wz - gtTwist.wz));
        std::cout << "Angular velocity error: " << angVelError << " rad/s\n";
        ASSERT_LT_(angVelError, MAXIMUM_VELOCITY_ERROR);

        std::cout << "Estimated twist: vx=" << estimatedTwist.vx << " vy=" << estimatedTwist.vy
                  << " vz=" << estimatedTwist.vz << " wx=" << estimatedTwist.wx
                  << " wy=" << estimatedTwist.wy << " wz=" << estimatedTwist.wz << "\n";
        std::cout << "Ground truth twist: vx=" << gtTwist.vx << " vy=" << gtTwist.vy
                  << " vz=" << gtTwist.vz << " wx=" << gtTwist.wx << " wy=" << gtTwist.wy
                  << " wz=" << gtTwist.wz << "\n";
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    int           numErrors          = 0;
    constexpr int RANDOM_REPETITIONS = 3;

    rng.randomize(9876);

    // Define test cases with different motion patterns
    std::vector<TestCase> tests = {
        // Test 1: Pure forward motion (no rotation)
        {Pose::Identity(), Twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),  // 1 m/s forward
         "Pure forward motion (1 m/s)"},

        // Test 2: Forward motion with yaw rotation (circular motion)
        {Pose::Identity(), Twist(1.5, 0.0, 0.0, 0.0, 0.0, 0.5),  // 1.5 m/s forward, 0.5 rad/s yaw
         "Circular motion (1.5 m/s forward, 0.5 rad/s yaw)"},

        // Test 3: Lateral motion (sideways)
        {Pose::FromXYZYawPitchRoll(2.0, 3.0, 0.0, 45.0_deg, 0.0, 0.0),
         Twist(0.0, 0.8, 0.0, 0.0, 0.0, 0.0),  // 0.8 m/s lateral
         "Lateral motion (0.8 m/s sideways)"},

        // Test 4: Combined forward and lateral motion
        {Pose::Identity(), Twist(1.0, 0.5, 0.0, 0.0, 0.0, 0.0),  // 1 m/s forward, 0.5 m/s lateral
         "Combined forward+lateral motion"},

        // Test 5: Forward motion with vertical component
        {Pose::FromTranslation(1.0, 2.0, 5.0),
         Twist(1.2, 0.0, 0.3, 0.0, 0.0, 0.0),  // 1.2 m/s forward, 0.3 m/s up
         "Forward motion with climb (vz=0.3)"},

        // Test 6: Rotation in place (pure yaw)
        {Pose::FromXYZYawPitchRoll(5.0, 5.0, 0.0, 30.0_deg, 0.0, 0.0),
         Twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.8),  // 0.8 rad/s yaw only
         "Rotation in place (pure yaw, 0.8 rad/s)"},

        // Test 7: Complex 3D motion
        {Pose::FromXYZYawPitchRoll(3.0, 4.0, 2.0, 60.0_deg, 5.0_deg, 0.0),
         Twist(1.0, 0.3, 0.2, 0.1, 0.05, 0.3),  // All components
         "Complex 3D motion (all velocity components)"},

        // Test 8: Slow motion
        {Pose::Identity(), Twist(0.2, 0.0, 0.0, 0.0, 0.0, 0.1),  // Slow: 0.2 m/s, 0.1 rad/s
         "Slow motion (0.2 m/s forward, 0.1 rad/s yaw)"},

        // Test 9: Fast motion with tight turn
        {Pose::FromXYZYawPitchRoll(10.0, 5.0, 1.0, -30.0_deg, 0.0, 0.0),
         Twist(2.5, 0.0, 0.0, 0.0, 0.0, 1.2),  // 2.5 m/s forward, 1.2 rad/s yaw
         "Fast motion with tight turn (2.5 m/s, 1.2 rad/s)"},

        // Test 10: Backward motion
        {Pose::FromTranslation(8.0, 6.0, 0.5),
         Twist(-0.8, 0.0, 0.0, 0.0, 0.0, 0.0),  // -0.8 m/s (backward)
         "Backward motion (-0.8 m/s)"},

        // Test 11: Crabbing motion (diagonal)
        {Pose::Identity(), Twist(0.7, 0.7, 0.0, 0.0, 0.0, 0.0),  // Diagonal motion
         "Crabbing/diagonal motion (vx=vy=0.7)"},

        // Test 12: 3D rotation (pitch and roll)
        {Pose::FromXYZYawPitchRoll(2.0, 2.0, 3.0, 0.0, 10.0_deg, 5.0_deg),
         Twist(0.5, 0.0, 0.0, 0.2, 0.1, 0.0),  // Forward with pitch/roll rotation
         "3D rotation with forward motion (wx=0.2, wy=0.1)"},

        // Test 13: Vertical motion
        {Pose::FromXYZYawPitchRoll(0.0, 0.0, 0.0, 0.0, 0.0_deg, .0_deg),
         Twist(0, 0.0, 0.1, 0.0, 0.0, 0.0), "Pure vertical motion"},
    };

    for (size_t testIdx = 0; testIdx < tests.size(); testIdx++)
    {
        const auto& t = tests[testIdx];

        for (int rep = 0; rep < RANDOM_REPETITIONS; rep++)
        {
            std::cout
                << "\n"
                   "========================================================================\n"
                   "=== Running test "
                << (testIdx + 1) << "/" << tests.size() << " rep " << rep << "/"
                << RANDOM_REPETITIONS << "\n"
                << "=== " << t.description << "\n"
                << "=== Initial pose: " << t.initial_pose << "\n"
                << "=== Constant twist: vx=" << t.constant_twist.vx << " vy=" << t.constant_twist.vy
                << " vz=" << t.constant_twist.vz << " wx=" << t.constant_twist.wx
                << " wy=" << t.constant_twist.wy << " wz=" << t.constant_twist.wz << "\n"
                << "========================================================================\n";

            bool failed = true;
            try
            {
                run_test(t);
                failed = false;
            }
            catch (const std::exception& e)
            {
                mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::RED);
                std::cerr << " ERROR:\n" << e.what() << std::endl;
                mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::DEFAULT);
            }

            std::cout
                << "\n"
                   "========================================================================\n";
            if (failed)
            {
                numErrors++;
                std::cout << "❌ FAILED\n";
            }
            else
            {
                std::cout << "✅ SUCCESS\n";
            }
            std::cout
                << "========================================================================\n\n";
        }
    }

    std::cout << "\n\n RESULT: ✅ " << (tests.size() * RANDOM_REPETITIONS - numErrors)
              << " SUCCESS, ❌ " << numErrors << " FAILURES.\n";

    return numErrors == 0 ? 0 : 1;
}