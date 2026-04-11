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
 * @file   MeasuredGravityFactor.h
 * @brief  GTSAM factor for IMU observations of gravity-aligned orientation
 * @author Jose Luis Blanco Claraco
 * @date   Dec 9, 2025
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{
/**
 * GTSAM factor for IMU observations of gravity-aligned orientation.
 *
 * It tries to optimize the pose such as normalized accelerometer vector points up (+Z).
 *
 */
class MeasuredGravityFactor
    : public gtsam::ExpressionFactorN<
          gtsam::Vector3 /*return type*/, gtsam::Pose3 /*T_enu2map*/, gtsam::Pose3 /*Pi*/
          >
{
   private:
    using This = MeasuredGravityFactor;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Vector3 /*return type*/, gtsam::Pose3 /*T_enu2map*/, gtsam::Pose3 /*Pi*/
        >;

    gtsam::Pose3  sensorPoseOnVehicle_ = gtsam::Pose3::Identity();
    gtsam::Point3 observedNormalizedAcc_b_;

   public:
    /// default constructor
    MeasuredGravityFactor();

    MeasuredGravityFactor(
        gtsam::Key kT_enu2map, gtsam::Key kPi, const gtsam::Pose3& sensorPoseOnVehicle,
        const gtsam::Vector3& observedNormalizedAcc_b, const gtsam::SharedNoiseModel& model);

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override;

    // Return measurement expression
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override;

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;

   private:
#if GTSAM_USES_BOOST
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        // **IMPORTANT** We need to deserialize parameters before the base
        // class, since it calls expression() and we need all parameters ready
        // at that point.
        ar& BOOST_SERIALIZATION_NVP(measured_);
        ar& boost::serialization::make_nvp(
            "MeasuredGravityFactor", boost::serialization::base_object<Base>(*this));
    }
#endif
};

}  // namespace mola::factors
