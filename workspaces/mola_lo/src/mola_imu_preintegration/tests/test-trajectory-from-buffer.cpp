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

/**
 * @file   test-trajectory-from-buffer.cpp
 * @brief  Unit tests for trajectory_from_buffer()
 * @author Jose Luis Blanco Claraco
 * @date   Sep 16, 2025
 */

#include <mola_imu_preintegration/ImuIntegrationParams.h>
#include <mola_imu_preintegration/LocalVelocityBuffer.h>
#include <mola_imu_preintegration/trajectory_from_buffer.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>

using namespace mola::imu;
using mrpt::math::TVector3D;
using mrpt::poses::CPose3D;
using mrpt::poses::Lie::SO;
using mrpt::random::getRandomGenerator;

const double IMU_RATE = 100.0;  // 100 Hz
const double IMU_DT   = 1.0 / IMU_RATE;

const double NOISE_STD_ORIENT  = 0.002;  // radians
const double NOISE_STD_LIN_VEL = 0.005;  // m/s
const double NOISE_STD_ACC     = 0.0005;  // m/s^2
const double NOISE_STD_GYRO    = 0.0001;  // rad/s

constexpr double TOL_POS = 0.2;  // meters
constexpr double TOL_ANG = 0.05;  // radians
constexpr double TOL_VEL = 0.1;  // m/s

// Helper function to generate ground truth trajectories
Trajectory generate_gt_trajectory(
    double duration, const std::function<TrajectoryPoint(double)>& func)
{
    Trajectory t;
    for (double time = 0.0; time <= duration; time += IMU_DT)
    {
        t[time] = func(time);
    }
    return t;
}

// Helper function to simulate IMU/sensor data and populate the buffer
void simulate_and_populate_buffer(
    LocalVelocityBuffer& buffer, const Trajectory& gt_traj, const ImuIntegrationParams& imu_params)
{
    auto& rnd = getRandomGenerator();

    // Add IMU data at 100 Hz
    for (const auto& [time, point] : gt_traj)
    {
        if (point.w_b && point.a_b)
        {
            TVector3D w_b_n;
            rnd.drawGaussian1DVector(w_b_n, 0, NOISE_STD_GYRO);

            TVector3D w_b_noisy = *point.w_b + imu_params.bias_gyro + w_b_n;

            TVector3D a_b_n;
            rnd.drawGaussian1DVector(a_b_n, 0, NOISE_STD_ACC);
            TVector3D a_b_noisy = *point.a_b + imu_params.bias_acc + a_b_n;

            buffer.add_angular_velocity(time, w_b_noisy);
            buffer.add_linear_acceleration(time, a_b_noisy);
        }
    }

    // Add global poses and velocities at 1 Hz
    for (const auto& [time, point] : gt_traj)
    {
        if (std::fmod(time, 1.0) < IMU_DT)
        {
            if (point.R_ga)
            {
                const std::array<double, 3> R_ga_n_arr = {
                    0, 0, rnd.drawGaussian1D(0, NOISE_STD_ORIENT)};
                const mrpt::math::CVectorFixedDouble<3> R_ga_n(R_ga_n_arr.data());

                // Add noise to global orientation
                mrpt::math::CMatrixDouble33 R_ga_noisy = *point.R_ga * SO<3>::exp(R_ga_n);
                buffer.add_orientation(time, R_ga_noisy);
            }
            if (point.v)
            {
                // Add noise to linear velocity
                TVector3D v_n;
                rnd.drawGaussian1DVector(v_n, 0, NOISE_STD_LIN_VEL);

                TVector3D v_noisy = *point.v + v_n;
                buffer.add_linear_velocity(time, v_noisy);
            }
        }
    }
}

// Test Case 1: Standing Still
void TrajectoryFromBuffer_StandingStill()
{
    MRPT_START

    ImuIntegrationParams imu_params;
    imu_params.gravity_vector = {0, 0, -9.81};

    // Ground truth: 2 seconds of standing still
    auto gt_func = [&]([[maybe_unused]] double time) -> TrajectoryPoint
    {
        TrajectoryPoint p;
        p.pose = CPose3D(0, 0, 0, 0, 0, 0);
        p.w_b  = {0, 0, 0};
        p.a_b  = {0, 0, 9.81};  // Sensor measures +9.81 due to gravity
        p.v    = {0, 0, 0};
        p.R_ga = mrpt::math::CMatrixDouble33::Identity();
        return p;
    };

    Trajectory          gt_traj = generate_gt_trajectory(2.0, gt_func);
    LocalVelocityBuffer buffer;
    buffer.set_reference_zero_time(0.0);
    buffer.parameters.max_time_window = 20.0;  // for this test, large window
    simulate_and_populate_buffer(buffer, gt_traj, imu_params);

    // Debug: buffer.toYAML().printAsYAML();

    // Reconstruct trajectory
    auto       samples            = buffer.collect_samples_around_reference_time(10.0);
    Trajectory reconstructed_traj = trajectory_from_buffer(samples, imu_params, false);

    // Debug: std::cout << trajectory_as_string(reconstructed_traj) << std::endl;

    // Verify
    for (const auto& [time, point] : reconstructed_traj)
    {
        ASSERT_NEAR_(point.pose.x(), 0.0, TOL_POS);
        ASSERT_NEAR_(point.pose.y(), 0.0, TOL_POS);
        ASSERT_NEAR_(point.pose.z(), 0.0, TOL_POS);
    }
    std::cout << "✅ TrajectoryFromBuffer_StandingStill passed!" << std::endl;
    MRPT_END
}

// Test Case 2: Constant Linear Velocity
void TrajectoryFromBuffer_ConstantLinearVelocity()
{
    MRPT_START

    ImuIntegrationParams imu_params;
    imu_params.gravity_vector = {0, 0, -9.81};
    TVector3D vel_gt          = {1.0, 0.5, 0.0};

    // Ground truth: 3 seconds of constant velocity
    auto gt_func = [&](double time) -> TrajectoryPoint
    {
        TrajectoryPoint p;
        p.pose = CPose3D(vel_gt.x * time, vel_gt.y * time, vel_gt.z * time, 0, 0, 0);
        p.w_b  = {0, 0, 0};
        p.a_b  = {0, 0, 9.81};
        p.v    = vel_gt;
        p.R_ga = mrpt::math::CMatrixDouble33::Identity();
        return p;
    };

    Trajectory          gt_traj = generate_gt_trajectory(3.0, gt_func);
    LocalVelocityBuffer buffer;
    buffer.set_reference_zero_time(0.0);
    buffer.parameters.max_time_window = 20.0;  // for this test, large window
    simulate_and_populate_buffer(buffer, gt_traj, imu_params);

    // Reconstruct trajectory
    auto       samples            = buffer.collect_samples_around_reference_time(10.0);
    Trajectory reconstructed_traj = trajectory_from_buffer(samples, imu_params, false);

    // Verify
    for (const auto& [time, point] : reconstructed_traj)
    {
        ASSERT_NEAR_(point.pose.x(), vel_gt.x * time, TOL_POS);
        ASSERT_NEAR_(point.pose.y(), vel_gt.y * time, TOL_POS);
        ASSERT_NEAR_(point.pose.z(), vel_gt.z * time, TOL_POS);

        ASSERT_NEAR_(point.v->x, vel_gt.x, TOL_VEL);
        ASSERT_NEAR_(point.v->y, vel_gt.y, TOL_VEL);
        ASSERT_NEAR_(point.v->z, vel_gt.z, TOL_VEL);
    }
    std::cout << "✅ TrajectoryFromBuffer_ConstantLinearVelocity passed!" << std::endl;
    MRPT_END
}

// Test Case: Constant Linear Velocity while rotating (around X: roll)
void TrajectoryFromBuffer_ConstantLinearVelocityWithRoll()
{
    MRPT_START

    ImuIntegrationParams imu_params;
    imu_params.gravity_vector = {0, 0, -9.81};
    TVector3D vel_gt          = {1.0, 0, 0.0};
    TVector3D ang_vel_gt      = {M_PI / 10.0, .0, .0};  // roll only

    // Ground truth:
    auto gt_func = [&](double time) -> TrajectoryPoint
    {
        TrajectoryPoint p;
        p.pose =
            CPose3D(vel_gt.x * time, vel_gt.y * time, vel_gt.z * time, 0, 0, ang_vel_gt.x * time);
        p.w_b  = ang_vel_gt;
        p.a_b  = p.pose.inverseRotateVector({0, 0, 9.81});
        p.v    = vel_gt;
        p.R_ga = p.pose.getRotationMatrix();
        return p;
    };

    Trajectory          gt_traj = generate_gt_trajectory(3.0, gt_func);
    LocalVelocityBuffer buffer;
    buffer.set_reference_zero_time(0.0);
    buffer.parameters.max_time_window = 20.0;  // for this test, large window
    simulate_and_populate_buffer(buffer, gt_traj, imu_params);

    // Reconstruct trajectory
    auto       samples            = buffer.collect_samples_around_reference_time(10.0);
    Trajectory reconstructed_traj = trajectory_from_buffer(samples, imu_params, false);

    // Verify
    for (const auto& [time, point] : reconstructed_traj)
    {
        ASSERT_NEAR_(point.pose.x(), vel_gt.x * time, TOL_POS);
        ASSERT_NEAR_(point.pose.y(), vel_gt.y * time, TOL_POS);
        ASSERT_NEAR_(point.pose.z(), vel_gt.z * time, TOL_POS);

        ASSERT_NEAR_(point.v->x, vel_gt.x, TOL_VEL);
        ASSERT_NEAR_(point.v->y, vel_gt.y, TOL_VEL);
        ASSERT_NEAR_(point.v->z, vel_gt.z, TOL_VEL);
    }
    std::cout << "✅ TrajectoryFromBuffer_ConstantLinearVelocityWithRoll passed!" << std::endl;
    MRPT_END
}

// Test Case: Perfect Circle (2D)
void TrajectoryFromBuffer_PerfectCircle()
{
    MRPT_START

    ImuIntegrationParams imu_params;
    imu_params.gravity_vector = {0, 0, -9.81};
    const double radius       = 2.0;
    const double ang_vel      = M_PI / 1.0;  // 2 sec per revolution

    // Ground truth: 20 seconds of circular motion
    auto gt_func = [&](double time) -> TrajectoryPoint
    {
        TrajectoryPoint p;
        double          angle = ang_vel * time;
        p.pose =
            CPose3D(radius * std::sin(angle), -radius * std::cos(angle) + radius, 0, angle, 0, 0);
        p.w_b  = {0, 0, ang_vel};
        p.a_b  = {0, ang_vel * ang_vel * radius, 9.81};  // Centripetal acceleration
        p.v    = {radius * ang_vel * std::cos(angle), radius * ang_vel * std::sin(angle), 0};
        p.R_ga = p.pose.getRotationMatrix();
        return p;
    };

    Trajectory          gt_traj = generate_gt_trajectory(4.0, gt_func);
    LocalVelocityBuffer buffer;
    buffer.set_reference_zero_time(0.0);
    buffer.parameters.max_time_window = 20.0;  // for this test, large window
    simulate_and_populate_buffer(buffer, gt_traj, imu_params);

    // Reconstruct trajectory
    auto       samples            = buffer.collect_samples_around_reference_time(20.0);
    Trajectory reconstructed_traj = trajectory_from_buffer(samples, imu_params, false);

    // Verify
    for (const auto& [time, point] : reconstructed_traj)
    {
        double angle = ang_vel * time;
        double x_gt  = radius * std::sin(angle);
        double y_gt  = -radius * std::cos(angle) + radius;

        ASSERT_NEAR_(point.pose.x(), x_gt, 0.5);
        ASSERT_NEAR_(point.pose.y(), y_gt, 0.5);
        ASSERT_NEAR_(point.pose.z(), 0.0, 0.5);
    }
    std::cout << "✅ TrajectoryFromBuffer_PerfectCircle passed!" << std::endl;
    MRPT_END
}

// Test Case 4: Spinning in 3D
void TrajectoryFromBuffer_Spinning3D()
{
    MRPT_START

    ImuIntegrationParams imu_params;
    imu_params.gravity_vector = {0, 0, -9.81};
    TVector3D ang_vel_gt      = {M_PI / 10.0, M_PI / 20.0, M_PI / 30.0};

    // Ground truth: 4 seconds of spinning without translation
    auto gt_func = [&](double time) -> TrajectoryPoint
    {
        TrajectoryPoint p;
        TVector3D       w_b_gt = ang_vel_gt;
        TVector3D       a_b_gt = {0, 0, 9.81};

        mrpt::math::CMatrixDouble33 R_pose_gt =
            SO<3>::exp((w_b_gt * time).asVector<mrpt::math::CVectorFixedDouble<3>>());
        p.pose.setRotationMatrix(R_pose_gt);

        p.w_b  = w_b_gt;
        p.a_b  = p.pose.inverseRotateVector(a_b_gt);
        p.v    = {0, 0, 0};
        p.R_ga = R_pose_gt;
        return p;
    };

    Trajectory          gt_traj = generate_gt_trajectory(4.0, gt_func);
    LocalVelocityBuffer buffer;
    buffer.set_reference_zero_time(0.0);
    buffer.parameters.max_time_window = 20.0;  // for this test, large window
    simulate_and_populate_buffer(buffer, gt_traj, imu_params);

    // Reconstruct trajectory
    auto       samples            = buffer.collect_samples_around_reference_time(10.0);
    Trajectory reconstructed_traj = trajectory_from_buffer(samples, imu_params, false);

    // Verify
    for (const auto& [time, point] : reconstructed_traj)
    {
        mrpt::math::CMatrixDouble33 R_gt =
            SO<3>::exp((ang_vel_gt * time).asVector<mrpt::math::CVectorFixedDouble<3>>());

        // print R_gt as cpose3d:
        CPose3D p_gt;
        p_gt.setRotationMatrix(R_gt);

        // Compare rotation matrices
        mrpt::math::CMatrixDouble33 diff_R   = R_gt.inverse() * point.pose.getRotationMatrix();
        const auto                  diff_log = SO<3>::log(diff_R);

        ASSERT_NEAR_(diff_log.norm(), 0.0, TOL_ANG);
        ASSERT_NEAR_(point.pose.x(), 0.0, TOL_POS);
        ASSERT_NEAR_(point.pose.y(), 0.0, TOL_POS);
        ASSERT_NEAR_(point.pose.z(), 0.0, TOL_POS);
    }

    std::cout << "✅ TrajectoryFromBuffer_Spinning3D passed!" << std::endl;

    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        TrajectoryFromBuffer_StandingStill();
        TrajectoryFromBuffer_ConstantLinearVelocity();
        TrajectoryFromBuffer_PerfectCircle();
        TrajectoryFromBuffer_ConstantLinearVelocityWithRoll();
        TrajectoryFromBuffer_Spinning3D();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Test failed: Unknown exception" << std::endl;
        return 1;
    }
}