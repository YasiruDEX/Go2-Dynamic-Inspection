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
 * @file   ImuInitialCalibrator.h
 * @brief  Provides a rough initial calibration and attitude for IMUs without
 *         proper bias calibration
 * @author Jose Luis Blanco Claraco
 * @date   Sep 5, 2025
 */

#pragma once

#include <mola_imu_preintegration/ImuTransformer.h>
#include <mrpt/obs/CObservationIMU.h>

#include <cstdlib>
#include <map>
#include <optional>

namespace mola::imu
{
/** Provides a rough initial calibration and attitude for IMUs without a proper
 *  bias calibration.
 *
 * \ingroup mola_imu_preintegration_grp
 */
class ImuInitialCalibrator
{
   public:
    ImuInitialCalibrator() = default;

    struct Parameters
    {
        Parameters() = default;

        std::size_t required_samples = 50;  //!< Minimum required samples
        double      max_samples_age  = .75;  //!< Maximum samples age [seconds]
        double      gravity          = 9.81;  //<! Gravity magnitude [m/s²]

        /** If provided by the IMU, prefer gravity-aligned orientation from the sensor instead of
         * accelerometer data. Should be normally preferable, so default is true.
         */
        bool use_imu_orientation = true;
    };

    Parameters parameters;

    /// Initial calibration results from getCalibration()
    struct Results
    {
        Results() = default;

        mrpt::math::TVector3D bias_gyro{0, 0, 0};  //!< Gyroscope bias
        mrpt::math::TVector3D bias_acc_b{0, 0, 0};  //!< Accelerometer bias, in base_link frame

        /// Gyroscope additive noise standard deviation
        mrpt::math::TVector3D noise_stddev_gyro{0, 0, 0};
        ///  Accelerometer bias, already in the base_link/body frame
        mrpt::math::TVector3D noise_stddev_acc{0, 0, 0};

        double pitch = 0;  //!< Estimated pitch angle, assuming being at rest during calibration
        double roll  = 0;  //!< Estimated roll angle, assuming being at rest during calibration

        /// Prints to a string a human-readable representation of all values
        std::string asString() const;
    };

    /// Inserts an IMU observation into the queue
    void add(const mrpt::obs::CObservationIMU::ConstPtr& obs);

    /// Returns true if there are already samples enough in the buffer to call getCalibration()
    [[nodiscard]] bool isReady() const;

    /// If enough samples are given, it computes the initial rough IMU calibration
    [[nodiscard]] std::optional<Results> getCalibration() const;

   private:
    std::map<std::string /*sensorLabel*/, ImuTransformer> imu_transformers_;

    /// Samples here have been already transformed to be on the base_link frame:
    std::map<double, const mrpt::obs::CObservationIMU> samples_;
};

// Remove when all distros have the version with "use_imu_orientation"
#define MOLA_IMU_PREINT_HAS_USE_IMU_ORIENT_PARAM

}  // namespace mola::imu
