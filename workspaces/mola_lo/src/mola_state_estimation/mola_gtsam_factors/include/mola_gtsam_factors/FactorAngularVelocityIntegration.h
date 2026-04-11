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
 * @file   FactorAngularVelocityIntegration.h
 * @brief  GTSAM factor for angular velocity integration
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2024
 *
 * This file implements a GTSAM factor that constrains rotation states based on
 * angular velocity measurements, using the exponential map for SO(3) integration.
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
 * @brief Factor for angular velocity integration using the SO(3) exponential map
 *
 * This factor implements the constraint:
 * \f[
 *   R_j = R_i \oplus \exp(\omega_i \cdot \Delta t)
 * \f]
 *
 * where:
 * - \f$R_i, R_j\f$ are 3D rotations (SO(3)) at times i and j
 * - \f$\omega_i\f$ is the angular velocity vector in the body frame at time i (rad/s)
 * - \f$\Delta t\f$ is the time interval between states
 * - \f$\exp\f$ is the exponential map from so(3) to SO(3)
 * - \f$\oplus\f$ is rotation composition
 *
 * The factor error is computed as:
 * \f[
 *   e = \log((R_i \oplus \exp(\omega_i \cdot \Delta t))^{-1} \cdot R_j)
 * \f]
 *
 * **Use Cases:**
 * - Integrating gyroscope measurements from IMUs
 * - Constraining orientation changes based on measured angular rates
 * - Smoothing rotation trajectories with velocity priors
 *
 * **Example Usage:**
 * @code
 * using namespace mola::factors;
 *
 * gtsam::NonlinearFactorGraph graph;
 *
 * // Keys for two rotation states and angular velocity
 * gtsam::Key kR0 = gtsam::Symbol('R', 0);
 * gtsam::Key kW0 = gtsam::Symbol('W', 0);  // Angular velocity at time 0
 * gtsam::Key kR1 = gtsam::Symbol('R', 1);
 *
 * // Noise model (uncertainty in rotation, typically from gyro noise)
 * auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01); // 0.01 rad std dev
 *
 * // Time step
 * double dt = 0.1; // 100 milliseconds
 *
 * // Add factor
 * graph.add(FactorAngularVelocityIntegration(kR0, kW0, kR1, dt, noise));
 * @endcode
 *
 * @see FactorAngularVelocityIntegrationPose for the Pose3 variant
 */
class FactorAngularVelocityIntegration : public gtsam::ExpressionFactorN<
                                             gtsam::Rot3 /*return type*/, gtsam::Rot3 /* Ri */,
                                             gtsam::Point3 /* bWi */, gtsam::Rot3 /* Rj */
                                             >
{
   private:
    using This = FactorAngularVelocityIntegration;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Rot3 /*return type*/, gtsam::Rot3 /* Ri */, gtsam::Point3 /* bWi */,
        gtsam::Rot3 /* Rj */
        >;

    double dt_ = .0;  ///< Time interval (seconds)

   public:
    /// Default constructor for serialization
    FactorAngularVelocityIntegration();

    /**
     * @brief Constructor
     * @param kRi Key for initial rotation state R_i
     * @param kbWi Key for angular velocity in body frame ω_i (rad/s)
     * @param kRj Key for final rotation state R_j
     * @param dt Time interval Δt between states (seconds)
     * @param model Noise model (3D for rotation error in tangent space)
     */
    FactorAngularVelocityIntegration(
        gtsam::Key kRi, gtsam::Key kbWi, gtsam::Key kRj, const double dt,
        const gtsam::SharedNoiseModel& model)
        : Base({kRi, kbWi, kRj}, model, gtsam::Rot3()), dt_(dt)
    {
        this->initialize(This::expression({kRi, kbWi, kRj}));
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

    /**
     * @brief Returns the measurement expression for this factor
     *
     * Implements: between(compose(Ri, Expmap(dt*bWi)), Rj)
     *
     * @param keys Array containing [kRi, kbWi, kRj]
     * @return Expression computing the rotation error
     */
    gtsam::Expression<gtsam::Rot3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Rot3_                     Ri(keys[0]);
        gtsam::Point3_                   bWi(keys[1]);
        gtsam::Rot3_                     Rj(keys[2]);
        gtsam::Expression<gtsam::Point3> deltaWi = dt_ * bWi;
        gtsam::Expression<gtsam::Rot3>   expmap_(&gtsam::Rot3::Expmap, deltaWi);

        return gtsam::between(gtsam::compose(Ri, expmap_), Rj);
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorAngularVelocityIntegration(" << keyFormatter(Factor::keys_[0])
                  << "," << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2])
                  << ")\n";
        gtsam::traits<gtsam::Rot3>::Print(measured_, "  measured: ");
        gtsam::traits<double>::Print(dt_, "  dt: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(dt_, e->dt_, tol);
    }

   private:
#if GTSAM_USES_BOOST
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        // **IMPORTANT** We need to deserialize parameters before the base
        // class, since it calls expression() and we need all parameters ready
        // at that point.
        ar& BOOST_SERIALIZATION_NVP(measured_);
        ar& BOOST_SERIALIZATION_NVP(dt_);
        ar& boost::serialization::make_nvp(
            "FactorAngularVelocityIntegration", boost::serialization::base_object<Base>(*this));
    }
#endif
};

/**
 * @brief Variant of FactorAngularVelocityIntegration using Pose3 states
 *
 * This factor is identical to FactorAngularVelocityIntegration but operates on
 * full 6-DOF poses (gtsam::Pose3) instead of pure rotations. Only the rotation
 * component is used; translation is ignored.
 *
 * **Use when:** Your state graph uses Pose3 instead of separate Rot3 variables
 *
 * @see FactorAngularVelocityIntegration for detailed mathematical description
 */
class FactorAngularVelocityIntegrationPose : public gtsam::ExpressionFactorN<
                                                 gtsam::Rot3 /*return type*/, gtsam::Pose3 /* Ti */,
                                                 gtsam::Point3 /* bWi */, gtsam::Pose3 /* Tj */
                                                 >
{
   private:
    using This = FactorAngularVelocityIntegrationPose;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Rot3 /*return type*/, gtsam::Pose3 /* Ti */, gtsam::Point3 /* bWi */,
        gtsam::Pose3 /* Tj */
        >;

    double dt_ = .0;  ///< Time interval (seconds)

   public:
    /// Default constructor for serialization
    FactorAngularVelocityIntegrationPose() = default;

    /**
     * @brief Constructor
     * @param kTi Key for initial pose (only rotation used)
     * @param kbWi Key for angular velocity in body frame
     * @param kTj Key for final pose (only rotation used)
     * @param dt Time interval between states
     * @param model Noise model (3D for rotation error)
     */
    FactorAngularVelocityIntegrationPose(
        gtsam::Key kTi, gtsam::Key kbWi, gtsam::Key kTj, const double dt,
        const gtsam::SharedNoiseModel& model)
        : Base({kTi, kbWi, kTj}, model, gtsam::Rot3()), dt_(dt)
    {
        this->initialize(This::expression({kTi, kbWi, kTj}));
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
    gtsam::Expression<gtsam::Rot3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Rot3_                     Ri = gtsam::rotation(gtsam::Pose3_(keys[0]));
        gtsam::Point3_                   bWi(keys[1]);
        gtsam::Rot3_                     Rj      = gtsam::rotation(gtsam::Pose3_(keys[2]));
        gtsam::Expression<gtsam::Point3> deltaWi = dt_ * bWi;
        gtsam::Expression<gtsam::Rot3>   expmap_(&gtsam::Rot3::Expmap, deltaWi);

        return gtsam::between(gtsam::compose(Ri, expmap_), Rj);
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorAngularVelocityIntegrationPose(" << keyFormatter(Factor::keys_[0])
                  << "," << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2])
                  << ")\n";
        gtsam::traits<gtsam::Rot3>::Print(measured_, "  measured: ");
        gtsam::traits<double>::Print(dt_, "  dt: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(dt_, e->dt_, tol);
    }

   private:
#if GTSAM_USES_BOOST
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        // **IMPORTANT** We need to deserialize parameters before the base
        // class, since it calls expression() and we need all parameters ready
        // at that point.
        ar& BOOST_SERIALIZATION_NVP(measured_);
        ar& BOOST_SERIALIZATION_NVP(dt_);
        ar& boost::serialization::make_nvp(
            "FactorAngularVelocityIntegrationPose", boost::serialization::base_object<Base>(*this));
    }
#endif
};

}  // namespace mola::factors