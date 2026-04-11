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
 * @file   test-navstate-odom-gnss-fusion.cpp
 * @brief  Unit tests for StateEstimationSmoother
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2025
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mola_state_estimation_smoother/pose_pdf_to_string_with_sigmas.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{

constexpr double GNSS_NOISE_XY_M    = 0.05;
constexpr double GNSS_NOISE_Z_M     = 0.05;
constexpr double MOTION_LIN_VX      = 1.0;  // m/s
constexpr double MOTION_ANG_WZ      = 0.5;  // rad/s
constexpr double ODOMETRY_NOISE_XY  = 0.01;
constexpr double ODOMETRY_NOISE_PHI = 0.1_deg;

constexpr double MAXIMUM_SE3_FINAL_ERROR = 0.40;

constexpr const char* ODOMETRY_NAME = "odom";

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

const size_t numPoses = 80;
const double T        = 0.1;  // sensors period

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
    # Options: KinematicModel::ConstantVelocity, KinematicModel::Tricycle
    kinematic_model: KinematicModel::ConstantVelocity

    # Time window to keep past observations in the filter [seconds]
    sliding_window_length: 4.0
    
    # Minimum time difference between frames to create a new frame [seconds]
    min_time_difference_to_create_new_frame: 0.01

    # Random walk model for linear acceleration uncertainty [m/s²]
    sigma_random_walk_acceleration_linear: 1.0

    # Random walk model for angular acceleration uncertainty [rad/s²]
    sigma_random_walk_acceleration_angular: 1.0

    # Integrator uncertainty for position [m]
    sigma_integrator_position: 0.10

    # Integrator uncertainty for orientation [rad]
    sigma_integrator_orientation: 0.10

    # Enable estimation of geo-referencing from GNSS and other sensors
    # If false, geo-reference must be provided externally or via fixed_geo_reference
    estimate_geo_reference: true

    # Fixed geo-reference to use when estimate_geo_reference is false
    #fixed_geo_reference: { latitude_deg: 0.0, longitude_deg: 0.0, altitude: 0.0 }

    # Link the "map" frame origin with "odom" for this toy example.
    # Otherwise, "odom" would "float" around without any particular XYZ known displacement.
    link_first_pose_to_reference_origin_sigma: 1e-6

)###";

using Pose      = mrpt::poses::CPose3D;
using Kinematic = mola::state_estimation_smoother::KinematicModel;

struct TestCase
{
    bool      has_gnss = true;
    Pose      pose;
    Kinematic model = Kinematic::ConstantVelocity;
};

// Test: simulate a random trajectory on XY on a given latitude/longitude with arbitrary initial
// heading, then generate noisy local odometry measurements, and noisy GNSS measurements and recover
// the geo-referenced trajectory from them.
//
void run_test(const TestCase& testCase)
{
    const auto actualInitialPoseWrtMap = testCase.pose;
    const auto kinematicModel          = testCase.model;

    mola::state_estimation_smoother::StateEstimationSmoother stateEst;

    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.profiler_.enable();

    {
        auto cfgYaml = mrpt::containers::yaml::FromText(navStateParams);

        cfgYaml["params"]["kinematic_model"] = mrpt::typemeta::enum2str(kinematicModel);
        stateEst.initialize(cfgYaml);
    }

    mrpt::poses::CPose3D actualVehiclePose = actualInitialPoseWrtMap;  // wrt "map" frame
    mrpt::poses::CPose2D odometryPose      = mrpt::poses::CPose2D::Identity();  // wrt "odom" frame

    mrpt::topography::TGeodeticCoords actualVehicleInitialGeoCoords;
    actualVehicleInitialGeoCoords.lat    = 4.0;
    actualVehicleInitialGeoCoords.lon    = 3.0;
    actualVehicleInitialGeoCoords.height = 50.0;

    for (size_t i = 0; i <= numPoses; i++)
    {
        // Generate a increment in motion:
        const double t = T * static_cast<double>(i);

        // 1. Simulate Trajectory (Circular motion)
        if (i > 0)
        {
            // Increment in local frame

            auto deltaPose2D     = mrpt::poses::CPose2D(MOTION_LIN_VX * T, 0.0, MOTION_ANG_WZ * T);
            const auto deltaPose = mrpt::poses::CPose3D(deltaPose2D);
            actualVehiclePose    = actualVehiclePose + deltaPose;

            deltaPose2D.x_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_XY));
            deltaPose2D.y_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_XY));
            deltaPose2D.phi_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_PHI));

            odometryPose = odometryPose + deltaPose2D;
        }

        // 2. Simulate noisy odometry:
        mrpt::obs::CObservationOdometry obsOdo;
        obsOdo.timestamp   = mrpt::Clock::fromDouble(t);
        obsOdo.sensorLabel = ODOMETRY_NAME;  // actually unused

        // Add realistic noise to odometry (accumulating drift)
        // Note: In a real scenario, noise is incremental. Here we just add noise
        // to the absolute GT to simulate a drifting input.
        auto noisyOdoPose = odometryPose;

        obsOdo.odometry        = noisyOdoPose;
        obsOdo.hasEncodersInfo = false;
        obsOdo.hasVelocities   = false;
        // Send to state estimator:
        stateEst.fuse_odometry(obsOdo, ODOMETRY_NAME);

        // 3. Simulate noisy GNSS:
        if (testCase.has_gnss)
        {
            mrpt::obs::CObservationGPS obsGps;
            obsGps.timestamp   = mrpt::Clock::fromDouble(t);
            obsGps.sensorLabel = "gnss";

            // Convert current Local ENU pose to Geodetic (Lat/Lon/Alt)
            mrpt::topography::TGeocentricCoords currentGeocentricCoords;
            mrpt::topography::ENUToGeocentric(
                {actualVehiclePose.x(), actualVehiclePose.y(), actualVehiclePose.z()},
                actualVehicleInitialGeoCoords, currentGeocentricCoords,
                mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

            mrpt::topography::TGeodeticCoords currentGeoCoords;
            mrpt::topography::geocentricToGeodetic(currentGeocentricCoords, currentGeoCoords);

            // sanity/validation check for geodetic -> ENU conversion:
            {
                mrpt::math::TPoint3D out_ENU_point;
                mrpt::topography::geodeticToENU_WGS84(
                    currentGeoCoords, out_ENU_point, actualVehicleInitialGeoCoords);
                ASSERT_NEAR_(out_ENU_point.x, actualVehiclePose.x(), 1e-2);
                ASSERT_NEAR_(out_ENU_point.y, actualVehiclePose.y(), 1e-2);
                ASSERT_NEAR_(out_ENU_point.z, actualVehiclePose.z(), 1e-2);
            }

            // Add noise to GNSS:
            constexpr double gnss_noise_deg = GNSS_NOISE_XY_M * mrpt::RAD2DEG(1.0 / 6300e3);

            mrpt::obs::gnss::Message_NMEA_GGA gga_msg;
            gga_msg.fields.latitude_degrees =
                currentGeoCoords.lat + rng.drawGaussian1D(0, gnss_noise_deg);
            gga_msg.fields.longitude_degrees =
                currentGeoCoords.lon + rng.drawGaussian1D(0, gnss_noise_deg);
            gga_msg.fields.altitude_meters =
                currentGeoCoords.height + rng.drawGaussian1D(0, GNSS_NOISE_Z_M);
            gga_msg.fields.fix_quality = 1;
            obsGps.setMsg(gga_msg);

            // Set GNSS Covariance (in meters)
            auto& cov = obsGps.covariance_enu.emplace();
            cov.setIdentity();
            cov(0, 0) = cov(1, 1) = mrpt::square(GNSS_NOISE_XY_M);
            cov(2, 2)             = mrpt::square(GNSS_NOISE_Z_M);

            // Send to state estimator:
            // Can use: stateEst.fuse_gnss(obsGps);
            // But let's use the Raw Data Consumer API to get it covered in tests:
            stateEst.onNewObservation(std::make_shared<const mrpt::obs::CObservationGPS>(obsGps));
        }

        // Enforce updating estimation from time to time:
        if (i % 5 == 0)
        {
            const auto stateOpt = stateEst.estimated_navstate(
                mrpt::Clock::fromDouble(t), stateEst.parameters().reference_frame_name);
            if (stateOpt && VERBOSE)
            {
                std::cout << stateOpt->asString() << "\n"
                          << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(
                                 stateOpt->pose)
                          << "\nGT: " << actualVehiclePose << "\n\n";
            }
        }
    }

    // Recover pose, in the reference frame:
    const double last_t = T * static_cast<double>(numPoses);
    {
        const auto stateOpt = stateEst.estimated_navstate(
            mrpt::Clock::fromDouble(last_t), stateEst.parameters().reference_frame_name);

        ASSERT_(stateOpt.has_value());
        std::cout << "State (ref.frame): " << stateOpt->asString() << "\n";

        // wrt map:
        const auto estimatedPoseWrtMap = stateOpt->pose.mean;

        const double final_se3_error =
            mrpt::poses::Lie::SE<3>::log(
                estimatedPoseWrtMap - (actualVehiclePose - actualInitialPoseWrtMap))
                .norm();
        std::cout << "final_se3_error: " << final_se3_error << "\n";
        ASSERT_LT_(final_se3_error, MAXIMUM_SE3_FINAL_ERROR);
    }

    // Recover pose, in the odometry frame:
    ASSERT_EQUAL_(stateEst.known_odometry_frame_ids().size(), 1);

    {
        const auto stateOpt =
            stateEst.estimated_navstate(mrpt::Clock::fromDouble(last_t), ODOMETRY_NAME);

        ASSERT_(stateOpt.has_value());
        std::cout << "State (odom frame): " << stateOpt->asString() << "\n";

        const double final_se3_error =
            mrpt::poses::Lie::SE<3>::log(
                stateOpt->pose.mean - (actualVehiclePose - actualInitialPoseWrtMap))
                .norm();
        std::cout << "final_se3_error: " << final_se3_error << "\n";
        ASSERT_LT_(final_se3_error, MAXIMUM_SE3_FINAL_ERROR);
    }

    // Request all estimated frames:
    auto T_enu_to_map = stateEst.estimated_T_enu_to_map();
    ASSERT_(T_enu_to_map.has_value());
    std::cout << "T_enu_to_map:\n"
              << mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(*T_enu_to_map)
              << "\n";
    {
        const auto covDet = std::sqrt(T_enu_to_map->cov.det());
        if (VERBOSE)
        {
            printf("√|cov(T_enu_to_map)|=%.03g\n", covDet);
        }
        if (testCase.has_gnss)
        {
            // We should have converged:
            ASSERT_LT_(covDet, 1.0e-3);

            // And to a valid global orientation (translation is arbitrary/not as important):
            const double enu2map_rot_error =
                mrpt::poses::Lie::SO<3>::log(
                    (T_enu_to_map->mean - testCase.pose).getRotationMatrix())
                    .norm();
            if (VERBOSE)
            {
                printf("log|error(T_enu_to_map)|=%.03g\n", enu2map_rot_error);
            }
            ASSERT_LT_(enu2map_rot_error, 0.05);
        }
        else
        {
            // We have no clue about geo-referencing since we don't have GNSS:
            ASSERT_GT_(covDet, 1.0e3);
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

    // Check if geo-referencing has converged:
    if (testCase.has_gnss)
    {
        mrpt::poses::CPose3DPDFGaussian veh_pose_in_map;
        bool hasConverged = stateEst.has_converged_localization(veh_pose_in_map);
        ASSERT_(hasConverged);
    }

    // done.
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    int numErrors  = 0;
    int numSuccess = 0;

    constexpr int RANDOM_REPETITIONS = 5;

    // shortcuts:
    for (const auto& kinModel : {Kinematic::ConstantVelocity, Kinematic::Tricycle})
    {
        std::vector<TestCase> tests = {
            {false, Pose::Identity(), kinModel},
            {false, Pose::FromTranslation(3.0, 2.0, 1.0), kinModel},
            {false, Pose::FromXYZYawPitchRoll(1.0, 3.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg), kinModel},
            {true, Pose::Identity(), kinModel},
            {true, Pose::FromTranslation(3.0, 2.0, 1.0), kinModel},
            {true, Pose::FromXYZYawPitchRoll(1.0, 3.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg), kinModel},
            {true, Pose::FromXYZYawPitchRoll(1.0, 3.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg), kinModel},
        };

        for (const auto& t : tests)
        {
            for (int rep = 0; rep < RANDOM_REPETITIONS; rep++)
            {
                rng.randomize(1234 + rep);  // for comparable results against diff kin models

                std::cout
                    << "\n"
                       "========================================================================\n"
                       "=== Running "
                    << rep << "/" << RANDOM_REPETITIONS << " test for initial pose: " << t.pose
                    << " Kinematic: " << mrpt::typemeta::enum2str(t.model)
                    << " has_gnss: " << t.has_gnss
                    << "\n"
                       "========================================================================\n";

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
                    mrpt::system::consoleColorAndStyle(
                        mrpt::system::ConsoleForegroundColor::DEFAULT);
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
                    numSuccess++;
                    std::cout << "✅ SUCCESS\n";
                }
                std::cout << "====================================================================="
                             "===\n\n";
            }
        }
    }

    std::cout << "\n\n RESULT: ✅ " << numSuccess << " SUCCESS, ❌ " << numErrors << " FAILURES.\n";

    return numErrors == 0 ? 0 : 1;
}