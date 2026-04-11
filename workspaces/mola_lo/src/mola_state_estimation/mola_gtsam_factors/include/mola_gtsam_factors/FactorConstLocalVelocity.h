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
 * @file   FactorConstLocalVelocity.h
 * @brief  GTSAM factor
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2024
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{
/**
 * Factor for constant velocity model in local coordinates, equivalent to
 * expression:
 *
 *   gtsam::rotate(Ri, bWi) - gtsam::rotate(Rj, bWj) = errZero
 *
 * This works for both, linear and angular velocities.
 *
 * Note that angular and linear velocities are stored in Values in the body "b"
 * frame, hence the "b" prefix, and the need for the orientations "R".
 */
class FactorConstLocalVelocity
    : public gtsam::ExpressionFactorN<
          gtsam::Point3 /*return type*/, gtsam::Rot3, gtsam::Point3, gtsam::Rot3, gtsam::Point3>
{
   private:
    using This = FactorConstLocalVelocity;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3, gtsam::Rot3, gtsam::Point3, gtsam::Rot3, gtsam::Point3>;

   public:
    /// default constructor
    FactorConstLocalVelocity();

    FactorConstLocalVelocity(
        gtsam::Key kRi, gtsam::Key kWi, gtsam::Key kRj, gtsam::Key kWj,
        const gtsam::SharedNoiseModel& model)
        : Base({kRi, kWi, kRj, kWj}, model, {0, 0, 0})
    {
        this->initialize(This::expression({kRi, kWi, kRj, kWj}));
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<This>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    // Return measurement expression
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Expression<gtsam::Rot3>   Ri_(keys[0]);
        gtsam::Expression<gtsam::Point3> bWi_(keys[1]);
        gtsam::Expression<gtsam::Rot3>   Rj_(keys[2]);
        gtsam::Expression<gtsam::Point3> bWj_(keys[3]);
        return {gtsam::rotate(Ri_, bWi_) - gtsam::rotate(Rj_, bWj_)};
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorConstLocalVelocity(" << keyFormatter(Factor::keys_[0]) << ","
                  << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2]) << ","
                  << keyFormatter(Factor::keys_[3]) << ")\n";
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol);
    }

    /** implement functions needed to derive from Factor */

    /** number of variables attached to this factor */
    // std::size_t size() const;

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
            "FactorConstLocalVelocity", boost::serialization::base_object<Base>(*this));
    }
#endif
};

/**
 * Factor for constant velocity model in local coordinates, equivalent to
 * expression:
 *
 *   gtsam::rotate(Ri, bWi) - gtsam::rotate(Rj, bWj) = errZero
 *
 * This works for both, linear and angular velocities.
 *
 * Note that angular and linear velocities are stored in Values in the body "b"
 * frame, hence the "b" prefix, and the need for the orientations "R".
 */
class FactorConstLocalVelocityPose
    : public gtsam::ExpressionFactorN<
          gtsam::Point3 /*return type*/, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3>
{
   private:
    using This = FactorConstLocalVelocityPose;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3>;

   public:
    /// default constructor
    FactorConstLocalVelocityPose() = default;

    FactorConstLocalVelocityPose(
        gtsam::Key kTi, gtsam::Key kWi, gtsam::Key kTj, gtsam::Key kWj,
        const gtsam::SharedNoiseModel& model)
        : Base({kTi, kWi, kTj, kWj}, model, {0, 0, 0})
    {
        this->initialize(This::expression({kTi, kWi, kTj, kWj}));
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<This>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    // Return measurement expression
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Expression<gtsam::Rot3>   Ri_ = gtsam::rotation(gtsam::Pose3_(keys[0]));
        gtsam::Expression<gtsam::Point3> bWi_(keys[1]);
        gtsam::Expression<gtsam::Rot3>   Rj_ = gtsam::rotation(gtsam::Pose3_(keys[2]));
        gtsam::Expression<gtsam::Point3> bWj_(keys[3]);
        return {gtsam::rotate(Ri_, bWi_) - gtsam::rotate(Rj_, bWj_)};
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorConstLocalVelocityPose(" << keyFormatter(Factor::keys_[0]) << ","
                  << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2]) << ","
                  << keyFormatter(Factor::keys_[3]) << ")\n";
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol);
    }

    /** implement functions needed to derive from Factor */

    /** number of variables attached to this factor */
    // std::size_t size() const;

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
            "FactorConstLocalVelocityPose", boost::serialization::base_object<Base>(*this));
    }
#endif
};

}  // namespace mola::factors
