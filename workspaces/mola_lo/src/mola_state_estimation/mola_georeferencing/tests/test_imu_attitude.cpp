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

#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_imu_preintegration/ImuTransformer.h>
#include <mola_imu_preintegration/LocalVelocityBuffer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

namespace
{
void run_single_test(double roll_deg, double pitch_deg, double sensor_yaw_deg, bool use_yaml)
{
    std::cout << "==================================================\n";
    std::cout << "[Test] Roll: " << roll_deg << " | Pitch: " << pitch_deg
              << " | SensorYaw: " << sensor_yaw_deg << " | Mode: "
              << (use_yaml ? "YAML (LocalVelocityBuffer)" : "Direct (CObservationIMU)")
              << std::endl;

    mrpt::maps::CSimpleMap sm;

    // Ground truth transformation from ENU to Map
    const mrpt::poses::CPose3D T_enu_to_map(
        0, 0, 0, 0, mrpt::DEG2RAD(pitch_deg), mrpt::DEG2RAD(roll_deg));

    // Sensor pose relative to vehicle
    const mrpt::poses::CPose3D T_veh_to_sensor(0, 0, 0, mrpt::DEG2RAD(sensor_yaw_deg), 0, 0);

    // True gravity vector in ENU
    const mrpt::math::TVector3D g_enu(0, 0, 9.81);

    for (int i = 0; i < 5; i++)
    {
        // Vehicle moving along the local map X-axis
        const mrpt::poses::CPose3D T_map_to_veh(i * 1.5, 0, 0, 0, 0, 0);

        auto pose_pdf  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose_pdf->mean = T_map_to_veh;

        auto sf = mrpt::obs::CSensoryFrame::Create();

        auto obs_imu        = mrpt::obs::CObservationIMU::Create();
        obs_imu->sensorPose = T_veh_to_sensor;
        const auto w_sensor = mrpt::math::TVector3D(0, 0, 0);  // No rotation

        {
            // Calculate acceleration in the SENSOR frame
            const mrpt::poses::CPose3D T_enu_to_sensor =
                T_enu_to_map + T_map_to_veh + T_veh_to_sensor;
            const mrpt::math::TVector3D a_sensor = T_enu_to_sensor.inverseRotateVector(g_enu);

            obs_imu->set(mrpt::obs::IMU_X_ACC, a_sensor.x);
            obs_imu->set(mrpt::obs::IMU_Y_ACC, a_sensor.y);
            obs_imu->set(mrpt::obs::IMU_Z_ACC, a_sensor.z);
            obs_imu->set(mrpt::obs::IMU_WX, w_sensor.x);
            obs_imu->set(mrpt::obs::IMU_WY, w_sensor.y);
            obs_imu->set(mrpt::obs::IMU_WZ, w_sensor.z);
        }

        if (!use_yaml)
        {
            // Acceleration in the SENSOR frame
            sf->insert(obs_imu);
        }
        else
        {
            // Calculate acceleration in the VEHICLE frame (expected by the buffer)
            mola::imu::ImuTransformer   imu_transformer;
            const auto                  imu_veh = imu_transformer.process(*obs_imu);
            const mrpt::math::TVector3D a_veh(
                imu_veh.get(mrpt::obs::IMU_X_ACC), imu_veh.get(mrpt::obs::IMU_Y_ACC),
                imu_veh.get(mrpt::obs::IMU_Z_ACC));

            mola::imu::LocalVelocityBuffer lvb;
            lvb.add_linear_acceleration(0.0, {a_veh.x, a_veh.y, a_veh.z});
            lvb.add_angular_velocity(0.0, w_sensor);  // No rotation

            auto                   comment = mrpt::obs::CObservationComment::Create();
            mrpt::containers::yaml y_root;
            y_root["local_velocity_buffer"] = lvb.toYAML();

            std::stringstream ss;
            ss << y_root;
            comment->text = ss.str();
            sf->insert(comment);
        }

        sm.insert(pose_pdf, sf);
    }

    // Run the georeferencing
    mola::SMGeoReferencingParams params;
    params.useIMUGravityAlignment = true;
    auto out                      = mola::simplemap_georeference(sm, params);

    ASSERTMSG_(out.geo_ref.has_value(), "Georeferencing output should not be empty!");

    const auto& T_est = out.geo_ref->T_enu_to_map.mean;

    // Compare Z-axes (Up vectors) directly to avoid Euler Gimbal Lock fragility at 90 degrees
    const mrpt::math::TVector3D z_est  = T_est.rotateVector({0, 0, 1});
    const mrpt::math::TVector3D z_true = T_enu_to_map.rotateVector({0, 0, 1});

    double z_dot = z_est.x * z_true.x + z_est.y * z_true.y + z_est.z * z_true.z;
    z_dot        = std::max(-1.0, std::min(1.0, z_dot));  // Clamp for safety before acos
    const double angle_error_deg = mrpt::RAD2DEG(std::acos(z_dot));

    std::cout << "  Calculated Roll:  " << mrpt::RAD2DEG(T_est.roll()) << " deg\n";
    std::cout << "  Calculated Pitch: " << mrpt::RAD2DEG(T_est.pitch()) << " deg\n";
    std::cout << "  Calculated Yaw:   " << mrpt::RAD2DEG(T_est.yaw()) << " deg\n";
    std::cout << "  Z-Axis Alignment Error: " << angle_error_deg << " deg\n";

    // Allow a small optimization tolerance (e.g., 0.1 degrees)
    const double tol_deg = 0.1;
    ASSERT_LT_(angle_error_deg, tol_deg);
}
}  // namespace

int main()
{
    try
    {
        // Define our test cases: {roll_deg, pitch_deg}
        const std::vector<std::pair<double, double>> angle_cases = {
            {30.0, 0.0}, {0.0, 20.0}, {0.0, 70.0}};

        const std::vector<double> sensor_yaws  = {0.0, 45.0};
        const std::vector<bool>   yaml_modes   = {false, true};
        std::size_t               test_count   = 0;
        std::size_t               failed_count = 0;

        for (const auto& angles : angle_cases)
        {
            for (const double yaw : sensor_yaws)
            {
                for (const bool use_yaml : yaml_modes)
                {
                    test_count++;

                    try
                    {
                        run_single_test(angles.first, angles.second, yaw, use_yaml);
                    }
                    catch (const std::exception& e)
                    {
                        failed_count++;
                        std::cerr << "\n[Test Failed] Exception in test case (Roll: "
                                  << angles.first << ", Pitch: " << angles.second
                                  << ", SensorYaw: " << yaw
                                  << ", Mode: " << (use_yaml ? "YAML" : "Direct") << "):\n"
                                  << e.what() << std::endl;
                    }
                }
            }
        }

        if (failed_count > 0)
        {
            std::cerr << "\n[Summary] " << failed_count << " out of " << test_count
                      << " tests failed." << std::endl;
        }
        else
        {
            std::cout << "\n[Success] All IMU gravity alignment tests passed!" << std::endl;
        }
        return failed_count == 0 ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[Test Failed] Exception caught:\n" << e.what() << std::endl;
        return 1;
    }
}