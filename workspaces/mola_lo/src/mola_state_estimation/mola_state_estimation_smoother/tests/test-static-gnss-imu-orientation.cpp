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
 * @file   test-static-gnss-imu-orientation.cpp
 * @brief  Unit tests for StateEstimationSmoother with static vehicle,
 *         GNSS position and IMU orientation fusion
 * @author Jose Luis Blanco Claraco
 * @date   Dec 09, 2025
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mola_state_estimation_smoother/pose_pdf_to_string_with_sigmas.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{

constexpr double GNSS_NOISE_XY_M = 0.10;  // 10 cm horizontal noise
constexpr double GNSS_NOISE_Z_M  = 0.15;  // 15 cm vertical noise
constexpr double IMU_NOISE_QUAT  = 0.01;  // Small quaternion noise

constexpr double MAXIMUM_POSITION_ERROR = 0.25;  // meters
constexpr double MAXIMUM_HEADING_ERROR  = 3.0_deg;  // radians

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

const size_t numReadings = 30;  // Number of GNSS+IMU readings
const double T           = 0.2;  // sensors period (5 Hz)

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
    sliding_window_length: 8.0
    
    # Minimum time difference between frames to create a new frame [seconds]
    min_time_difference_to_create_new_frame: 0.05

    # Random walk model for linear acceleration uncertainty [m/s²]
    sigma_random_walk_acceleration_linear: 0.1

    # Random walk model for angular acceleration uncertainty [rad/s²]
    sigma_random_walk_acceleration_angular: 0.1

    # Integrator uncertainty for position [m]
    sigma_integrator_position: 0.02

    # Integrator uncertainty for orientation [rad]
    sigma_integrator_orientation: 0.02

    # Fixed geo-reference (vehicle is static, so we provide external reference)
    estimate_geo_reference: false
)###";

using Pose = mrpt::poses::CPose3D;

struct TestCase
{
    Pose        vehicle_pose_wrt_map;  // Ground truth pose
    double      latitude_deg;  // Fixed geo-reference latitude
    double      longitude_deg;  // Fixed geo-reference longitude
    double      altitude_m;  // Fixed geo-reference altitude
    double      imu_attitude_azimuth_offset_deg;  // IMU zero to Azimuth offset
    Pose        imu_sensor_pose;  // IMU mounting pose wrt vehicle base_link
    std::string description;  // Test case description
};

// Test: A vehicle is static at a known pose. We have a fixed geo-reference.
// Generate noisy GNSS readings (position) and noisy IMU readings (orientation).
// The IMU may have a non-trivial mounting pose wrt the vehicle base.
// Verify the estimator can recover the vehicle's pose (x, y, z, yaw, pitch, roll).
//
void run_test(const TestCase& testCase)
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;

    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.profiler_.enable();

    // Configure with fixed geo-reference
    {
        auto cfgYaml = mrpt::containers::yaml::FromText(navStateParams);

        // Set the fixed geo-reference
        cfgYaml["params"]["fixed_geo_reference"]["latitude_deg"]  = testCase.latitude_deg;
        cfgYaml["params"]["fixed_geo_reference"]["longitude_deg"] = testCase.longitude_deg;
        cfgYaml["params"]["fixed_geo_reference"]["altitude"]      = testCase.altitude_m;

        cfgYaml["params"]["imu_attitude_azimuth_offset_deg"] =
            testCase.imu_attitude_azimuth_offset_deg;

        stateEst.initialize(cfgYaml);
    }

    // Ground truth vehicle pose (static throughout the test)
    const auto actualVehiclePose = testCase.vehicle_pose_wrt_map;
    const auto imuSensorPose     = testCase.imu_sensor_pose;

    // Fixed geo-reference point (ENU origin)
    mrpt::topography::TGeodeticCoords geoReference;
    geoReference.lat    = testCase.latitude_deg;
    geoReference.lon    = testCase.longitude_deg;
    geoReference.height = testCase.altitude_m;

    // Convert vehicle pose to geodetic coordinates
    mrpt::topography::TGeocentricCoords vehicleGeocentric;
    mrpt::topography::ENUToGeocentric(
        {actualVehiclePose.x(), actualVehiclePose.y(), actualVehiclePose.z()}, geoReference,
        vehicleGeocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

    mrpt::topography::TGeodeticCoords vehicleGeodetic;
    mrpt::topography::geocentricToGeodetic(vehicleGeocentric, vehicleGeodetic);

    // Include Azimuth reference, such as IMU yaw=0 means Azimuth=0 (North),
    // which for ENU means -90 deg yaw:
    const auto vehicleToAzimuth = mrpt::poses::CPose3D::FromYawPitchRoll(
        mrpt::DEG2RAD(-90.0 - testCase.imu_attitude_azimuth_offset_deg), 0.0_deg, 0.0_deg);

    // Compute IMU pose in global frame: vehicle_pose (+) imu_sensor_pose
    const auto actualImuPoseGlobal = actualVehiclePose + vehicleToAzimuth + imuSensorPose;

    if (VERBOSE)
    {
        std::cout << "\n=== Test Configuration ===\n";
        std::cout << "Description: " << testCase.description << "\n";
        std::cout << "Ground truth vehicle pose: " << actualVehiclePose << "\n";
        std::cout << "IMU sensor pose (wrt vehicle): " << imuSensorPose << "\n";
        std::cout << "IMU pose (global): " << actualImuPoseGlobal << "\n";
        std::cout << "Geo-reference: lat=" << geoReference.lat << " lon=" << geoReference.lon
                  << " alt=" << geoReference.height << "\n";
        std::cout << "Vehicle geodetic: lat=" << vehicleGeodetic.lat
                  << " lon=" << vehicleGeodetic.lon << " alt=" << vehicleGeodetic.height << "\n\n";
    }

    // Generate and fuse GNSS + IMU readings
    for (size_t i = 0; i < numReadings; i++)
    {
        const double t = T * static_cast<double>(i);

        // 1. Simulate noisy GNSS reading
        {
            mrpt::obs::CObservationGPS obsGps;
            obsGps.timestamp   = mrpt::Clock::fromDouble(t);
            obsGps.sensorLabel = "gnss";

            // Add noise to GNSS position
            constexpr double gnss_noise_deg = GNSS_NOISE_XY_M * mrpt::RAD2DEG(1.0 / 6300e3);

            mrpt::obs::gnss::Message_NMEA_GGA gga_msg;
            gga_msg.fields.latitude_degrees =
                vehicleGeodetic.lat + rng.drawGaussian1D(0, gnss_noise_deg);
            gga_msg.fields.longitude_degrees =
                vehicleGeodetic.lon + rng.drawGaussian1D(0, gnss_noise_deg);
            gga_msg.fields.altitude_meters =
                vehicleGeodetic.height + rng.drawGaussian1D(0, GNSS_NOISE_Z_M);
            gga_msg.fields.fix_quality = 4;  // RTK fixed
            obsGps.setMsg(gga_msg);

            // Set GNSS Covariance (in meters)
            auto& cov = obsGps.covariance_enu.emplace();
            cov.setIdentity();
            cov(0, 0) = cov(1, 1) = mrpt::square(GNSS_NOISE_XY_M);
            cov(2, 2)             = mrpt::square(GNSS_NOISE_Z_M);

            // Send to state estimator
            stateEst.fuse_gnss(obsGps);

            if (VERBOSE && i % 5 == 0)
            {
                std::cout << "t=" << t << " GNSS: lat=" << gga_msg.fields.latitude_degrees
                          << " lon=" << gga_msg.fields.longitude_degrees
                          << " alt=" << gga_msg.fields.altitude_meters << "\n";
            }
        }

        // 2. Simulate noisy IMU orientation reading
        {
            mrpt::obs::CObservationIMU obsImu;
            obsImu.timestamp   = mrpt::Clock::fromDouble(t);
            obsImu.sensorLabel = "imu";

            // IMPORTANT: Set the IMU sensor pose relative to the vehicle
            obsImu.sensorPose = imuSensorPose;

            // Get ground truth IMU orientation (in global frame) as quaternion
            mrpt::math::CQuaternionDouble gtQuatGlobal;
            actualImuPoseGlobal.getAsQuaternion(gtQuatGlobal);

            // Add noise to quaternion components
            mrpt::math::CQuaternionDouble noisyQuat;
            for (int k = 0; k < 4; k++)
            {
                noisyQuat[k] = gtQuatGlobal[k] + rng.drawGaussian1D(0, IMU_NOISE_QUAT);
            }

            // Normalize quaternion after adding noise
            noisyQuat.normalize();

            // Set the orientation in the IMU observation
            obsImu.set(mrpt::obs::IMU_ORI_QUAT_W, noisyQuat.w());
            obsImu.set(mrpt::obs::IMU_ORI_QUAT_X, noisyQuat.x());
            obsImu.set(mrpt::obs::IMU_ORI_QUAT_Y, noisyQuat.y());
            obsImu.set(mrpt::obs::IMU_ORI_QUAT_Z, noisyQuat.z());

            // Set gravity-aligned acceleration (static vehicle)
            // Note: These are in the IMU sensor frame, not vehicle frame
            {
                const auto localUp = actualImuPoseGlobal.inverseRotateVector({0, 0, 9.81});

                obsImu.set(mrpt::obs::IMU_X_ACC, localUp.x + rng.drawGaussian1D(0, 0.1));
                obsImu.set(mrpt::obs::IMU_Y_ACC, localUp.y + rng.drawGaussian1D(0, 0.1));
                obsImu.set(mrpt::obs::IMU_Z_ACC, localUp.z + +rng.drawGaussian1D(0, 0.1));
            }

            // Set zero angular velocity (static vehicle)
            obsImu.set(mrpt::obs::IMU_WX, rng.drawGaussian1D(0, 0.01));
            obsImu.set(mrpt::obs::IMU_WY, rng.drawGaussian1D(0, 0.01));
            obsImu.set(mrpt::obs::IMU_WZ, rng.drawGaussian1D(0, 0.01));

            // Send to state estimator
            // Can use: stateEst.fuse_imu(obsImu);
            // But let's use the Raw Data Consumer API to get it covered in tests:
            stateEst.onNewObservation(std::make_shared<const mrpt::obs::CObservationIMU>(obsImu));

            if (VERBOSE && i % 5 == 0)
            {
                std::cout << "t=" << t << " IMU quat (global): [" << noisyQuat[0] << ", "
                          << noisyQuat[1] << ", " << noisyQuat[2] << ", " << noisyQuat[3] << "]\n";
            }
        }

        // Check estimation periodically
        if (i > 5 && i % 5 == 0)
        {
            const auto stateOpt = stateEst.estimated_navstate(
                mrpt::Clock::fromDouble(t), stateEst.parameters().reference_frame_name);

            if (stateOpt && VERBOSE)
            {
                const auto estimatedPose = stateOpt->pose.mean;
                std::cout << "\n--- Estimation at t=" << t << " ---\n";
                std::cout << "Estimated: " << estimatedPose << "\n";
                std::cout << "Ground truth: " << actualVehiclePose << "\n";
                std::cout << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(
                                 stateOpt->pose)
                          << "\n";

                // Compute errors
                const auto posError =
                    (estimatedPose.asTPose() - actualVehiclePose.asTPose()).norm();
                double yaw_est, pitch_est, roll_est;
                estimatedPose.getYawPitchRoll(yaw_est, pitch_est, roll_est);
                double yaw_gt, pitch_gt, roll_gt;
                actualVehiclePose.getYawPitchRoll(yaw_gt, pitch_gt, roll_gt);
                const auto headingError = std::abs(mrpt::math::angDistance(yaw_est, yaw_gt));

                std::cout << "Position error: " << posError << " m\n";
                std::cout << "Heading error: " << mrpt::RAD2DEG(headingError) << " deg\n\n";
            }
        }
    }

    // Final verification
    const double last_t = T * static_cast<double>(numReadings - 1);
    {
        const auto stateOpt = stateEst.estimated_navstate(
            mrpt::Clock::fromDouble(last_t), stateEst.parameters().reference_frame_name);

        ASSERT_(stateOpt.has_value());

        const auto estimatedPose = stateOpt->pose.mean;

        std::cout << "\n=== FINAL RESULTS ===\n";
        std::cout << "Estimated pose: " << estimatedPose << "\n";
        std::cout << "Ground truth pose: " << actualVehiclePose << "\n";
        std::cout << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(stateOpt->pose)
                  << "\n";

        // Check position error (xyz)
        const auto positionError = (estimatedPose.asTPose() - actualVehiclePose.asTPose()).norm();
        std::cout << "Position error (xyz): " << positionError << " m\n";
        ASSERT_LT_(positionError, MAXIMUM_POSITION_ERROR);

        // Check heading error (yaw)
        double yaw_est, pitch_est, roll_est;
        estimatedPose.getYawPitchRoll(yaw_est, pitch_est, roll_est);
        double yaw_gt, pitch_gt, roll_gt;
        actualVehiclePose.getYawPitchRoll(yaw_gt, pitch_gt, roll_gt);

        const auto headingError = std::abs(mrpt::math::angDistance(yaw_est, yaw_gt));
        std::cout << "Heading error (yaw): " << mrpt::RAD2DEG(headingError) << " deg\n";
        ASSERT_LT_(headingError, MAXIMUM_HEADING_ERROR);

        // Optional: Check pitch and roll errors
        const auto pitchError = std::abs(mrpt::math::angDistance(pitch_est, pitch_gt));
        const auto rollError  = std::abs(mrpt::math::angDistance(roll_est, roll_gt));
        std::cout << "Pitch error: " << mrpt::RAD2DEG(pitchError) << " deg\n";
        std::cout << "Roll error: " << mrpt::RAD2DEG(rollError) << " deg\n";
    }

    // Verify geo-referencing transform
    {
        auto T_enu_to_map = stateEst.estimated_T_enu_to_map();
        ASSERT_(T_enu_to_map.has_value());

        std::cout << "\nT_enu_to_map:\n"
                  << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(*T_enu_to_map)
                  << "\n";

        // With fixed geo-reference, the uncertainty should be very low
        const auto covDet = std::sqrt(T_enu_to_map->cov.det());
        std::cout << "√|cov(T_enu_to_map)|=" << covDet << "\n";
        ASSERT_LT_(covDet, 1.0e-6);  // Very low uncertainty
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    int           numErrors          = 0;
    constexpr int RANDOM_REPETITIONS = 3;

    rng.randomize(5678);

    // Define test cases with different vehicle poses, geo-references, and IMU mounting poses
    std::vector<TestCase> tests = {
        // Test 1: Vehicle at origin, IMU aligned with vehicle
        {Pose::Identity(), 36.8407, -2.4093, 100.0, 0.0, Pose::Identity(),
         "Vehicle at origin, IMU aligned with vehicle"},

        // Test 2: Vehicle displaced, IMU with small rotation (typical mounting)
        {Pose::FromTranslation(10.0, 20.0, 0.5), 36.8407, -2.4093, 100.0, 0.0,
         Pose::FromXYZYawPitchRoll(0.1, 0.0, 0.05, 5.0_deg, 2.0_deg, 1.0_deg),
         "Vehicle displaced, IMU with small rotation"},

        // Test 3: Vehicle with heading, IMU rotated 90° around Z (left-facing)
        {Pose::FromXYZYawPitchRoll(5.0, 15.0, 1.0, 30.0_deg, 0.0_deg, 0.0_deg), 36.8407, -2.4093,
         100.0, 0.0, Pose::FromXYZYawPitchRoll(0.0, 0.0, 0.0, 90.0_deg, 0.0_deg, 0.0_deg),
         "Vehicle with 30° heading, IMU rotated 90° left (Y-axis forward)"},

        // Test 4: Vehicle with pitch/roll, IMU rotated 180° around Z (rear-facing)
        {Pose::FromXYZYawPitchRoll(8.0, 12.0, 2.0, 45.0_deg, 5.0_deg, 3.0_deg), 36.8407, -2.4093,
         100.0, 0.0, Pose::FromXYZYawPitchRoll(0.0, 0.0, 0.1, 180.0_deg, 0.0_deg, 0.0_deg),
         "Vehicle with pitch/roll, IMU rotated 180° (rear-facing)"},

        // Test 5: Vehicle with heading, IMU rotated -90° around Z (right-facing)
        {Pose::FromXYZYawPitchRoll(3.0, 7.0, 0.8, -20.0_deg, 0.0_deg, 0.0_deg), 40.4168, -3.7038,
         650.0, 0.0, Pose::FromXYZYawPitchRoll(0.0, 0.0, 0.0, -90.0_deg, 0.0_deg, 0.0_deg),
         "Different geo-ref (Madrid), IMU rotated 90° right"},

        // Test 6: IMU mounted upside-down (180° roll)
        {Pose::FromXYZYawPitchRoll(2.0, 3.0, 0.5, 15.0_deg, 0.0_deg, 0.0_deg), 36.8407, -2.4093,
         100.0, 0.0, Pose::FromXYZYawPitchRoll(0.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 180.0_deg),
         "IMU mounted upside-down (180° roll)"},

        // Test 7: IMU with 90° pitch (vertical mounting, looking down)
        {Pose::FromXYZYawPitchRoll(6.0, 9.0, 1.5, 60.0_deg, 0.0_deg, 0.0_deg), 36.8407, -2.4093,
         100.0, 0.0, Pose::FromXYZYawPitchRoll(0.0, 0.0, 0.0, 0.0_deg, 90.0_deg, 0.0_deg),
         "IMU with 90° pitch (vertical, looking down)"},

        // Test 8: IMU with complex rotation (45° yaw, 30° pitch, 15° roll)
        {Pose::FromXYZYawPitchRoll(4.0, 8.0, 1.2, -45.0_deg, 2.0_deg, -1.0_deg), 36.8407, -2.4093,
         100.0, 0.0, Pose::FromXYZYawPitchRoll(0.05, -0.02, 0.08, 45.0_deg, 30.0_deg, 15.0_deg),
         "IMU with complex rotation (45° yaw, 30° pitch, 15° roll)"},

        // Test 9: IMU with large displacement and rotation
        {Pose::FromXYZYawPitchRoll(12.0, 18.0, 2.5, 75.0_deg, -3.0_deg, 2.0_deg), 36.8407, -2.4093,
         100.0, 0.0, Pose::FromXYZYawPitchRoll(0.5, 0.3, 0.2, 135.0_deg, -20.0_deg, 10.0_deg),
         "IMU with large displacement and complex rotation"},

        // Test 10: IMU with different azimuth reference
        {Pose::FromXYZYawPitchRoll(12.0, 18.0, 2.5, 75.0_deg, -3.0_deg, 2.0_deg), 36.8407, -2.4093,
         100.0, -90.0, Pose::FromXYZYawPitchRoll(0.5, 0.3, 0.2, 135.0_deg, -20.0_deg, 10.0_deg),
         "IMU with different azimuth reference"},
    };

    std::set<std::string> setSuccess;
    std::set<std::string> setFail;

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
                << "=== Vehicle pose: " << t.vehicle_pose_wrt_map << "\n"
                << "=== IMU sensor pose: " << t.imu_sensor_pose << "\n"
                << "=== Geo-ref: lat=" << t.latitude_deg << " lon=" << t.longitude_deg
                << " alt=" << t.altitude_m << "\n"
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
                setFail.insert(t.description);
            }
            else
            {
                std::cout << "✅ SUCCESS\n";
                setSuccess.insert(t.description);
            }
            std::cout
                << "========================================================================\n\n";
        }
    }

    std::cout << "SUCCESS: " << setSuccess.size() << "\n";
    for (const auto& t : setSuccess)
    {
        std::cout << "✅ - " << t << "\n";
    }
    std::cout << "\n";

    std::cout << "FAILURES: " << setFail.size() << "\n";
    for (const auto& t : setFail)
    {
        std::cout << "❌ - " << t << "\n";
    }
    std::cout << "\n";

    return numErrors == 0 ? 0 : 1;
}