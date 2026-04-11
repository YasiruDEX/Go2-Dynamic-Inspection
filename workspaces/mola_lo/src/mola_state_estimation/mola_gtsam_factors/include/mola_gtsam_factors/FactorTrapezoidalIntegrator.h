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
 * @file   FactorTrapezoidalIntegrator.h
 * @brief  GTSAM factor for velocity integration using trapezoidal rule
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2024
 *
 * This file implements numerical integration of velocities using the trapezoidal
 * rule, providing more accurate motion prediction than simple Euler integration.
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{

/**
 * @brief Factor implementing trapezoidal rule integration for velocity
 *
 * This factor constrains position states based on velocity measurements using
 * the trapezoidal integration rule, which provides second-order accuracy.
 *
 * **Mathematical Model:**
 * \f[
 *   p_j = p_i + \frac{\Delta t}{2} \left( R_i \cdot v_i + R_j \cdot v_j \right)
 * \f]
 *
 * where:
 * - \f$p_i, p_j\f$ are 3D positions at times i and j
 * - \f$v_i, v_j\f$ are velocity vectors in the **body frame**
 * - \f$R_i, R_j\f$ are orientation matrices (rotate body frame to global frame)
 * - \f$\Delta t\f$ is the time interval
 *
 * The factor error is:
 * \f[
 *   e = p_i + \frac{\Delta t}{2}(R_i v_i + R_j v_j) - p_j
 * \f]
 *
 * **Advantages over Euler Integration:**
 * - More accurate for varying velocities (2nd order vs 1st order)
 * - Better numerical stability for larger time steps
 * - Accounts for velocity changes during the interval
 *
 * **When to Use:**
 * - Smoothing trajectories with velocity estimates at each timestep
 * - Fusing IMU linear acceleration (integrated to velocity)
 * - Visual-inertial odometry
 * - Any scenario where velocities are available at both endpoints
 *
 * **Comparison:**
 * - **Euler integration**: Uses only v_i → simpler, less accurate
 * - **Trapezoidal integration**: Uses both v_i and v_j → more accurate, needs both velocities
 * - **RK4**: Even more accurate but computationally expensive
 *
 * **Example Usage:**
 * @code
 * using namespace mola::factors;
 *
 * gtsam::NonlinearFactorGraph graph;
 *
 * // State variables at time i
 * gtsam::Key kPi = gtsam::Symbol('p', 0);  // Position
 * gtsam::Key kVi = gtsam::Symbol('v', 0);  // Velocity (body frame)
 * gtsam::Key kRi = gtsam::Symbol('r', 0);  // Orientation
 *
 * // State variables at time j
 * gtsam::Key kPj = gtsam::Symbol('p', 1);
 * gtsam::Key kVj = gtsam::Symbol('v', 1);
 * gtsam::Key kRj = gtsam::Symbol('r', 1);
 *
 * // Noise model (3D position error)
 * auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);  // 10cm std dev
 *
 * double dt = 0.1; // 100ms
 *
 * // Add trapezoidal integration constraint
 * graph.add(FactorTrapezoidalIntegrator(
 *     kPi, kVi, kRi,
 *     kPj, kVj, kRj,
 *     dt, noise
 * ));
 * @endcode
 *
 * **Important Notes:**
 * - Velocities must be in the **body frame** (not global frame)
 * - Orientations R_i and R_j rotate from body to global frame
 * - For 2D motion, set vz = 0 in velocities
 * - Time step dt should be reasonably small (typically < 1 second)
 *
 * @see FactorTrapezoidalIntegratorPose for the Pose3 variant
 * @see FactorTricycleKinematic for kinematic models (Euler integration)
 */
class FactorTrapezoidalIntegrator : public gtsam::ExpressionFactorN<
                                        gtsam::Point3 /*return type*/,
                                        gtsam::Point3,  // Pi
                                        gtsam::Point3,  // bVi (body-frame velocity)
                                        gtsam::Rot3,  // Ri
                                        gtsam::Point3,  // Pj
                                        gtsam::Point3,  // bVj
                                        gtsam::Rot3  // Rj
                                        >
{
   private:
    using This = FactorTrapezoidalIntegrator;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3 /*return type*/, gtsam::Point3, gtsam::Point3, gtsam::Rot3, gtsam::Point3,
        gtsam::Point3, gtsam::Rot3>;

    double dt_ = .0;  ///< Time interval Δt (seconds)

   public:
    /// Default constructor for serialization
    FactorTrapezoidalIntegrator();

    /**
     * @brief Construct a trapezoidal integrator factor
     *
     * @param kPi Key for initial position p_i
     * @param kVi Key for initial velocity v_i (body frame, m/s)
     * @param kRi Key for initial orientation R_i
     * @param kPj Key for final position p_j
     * @param kVj Key for final velocity v_j (body frame, m/s)
     * @param kRj Key for final orientation R_j
     * @param dt Time interval Δt (seconds)
     * @param model Noise model (3D position error)
     */
    FactorTrapezoidalIntegrator(
        gtsam::Key kPi, gtsam::Key kVi, gtsam::Key kRi, gtsam::Key kPj, gtsam::Key kVj,
        gtsam::Key kRj, const double dt, const gtsam::SharedNoiseModel& model)
        : Base({kPi, kVi, kRi, kPj, kVj, kRj}, model, /* error=0 */ {0, 0, 0}), dt_(dt)
    {
        this->initialize(This::expression({kPi, kVi, kRi, kPj, kVj, kRj}));
    }

    /// @return Deep copy of this factor
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
     * @brief Measurement expression
     *
     * Implements: Pi + 0.5*dt*(rotate(Ri, bVi) + rotate(Rj, bVj)) - Pj
     *
     * @param keys Array containing [kPi, kVi, kRi, kPj, kVj, kRj]
     * @return Expression computing position integration error
     */
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Expression<gtsam::Point3> Pi_(keys[0]);
        gtsam::Expression<gtsam::Point3> bVi_(keys[1]);
        gtsam::Expression<gtsam::Rot3>   Ri_(keys[2]);

        gtsam::Expression<gtsam::Point3> Pj_(keys[3]);
        gtsam::Expression<gtsam::Point3> bVj_(keys[4]);
        gtsam::Expression<gtsam::Rot3>   Rj_(keys[5]);

        return {Pi_ + 0.5 * dt_ * (gtsam::rotate(Ri_, bVi_) + gtsam::rotate(Rj_, bVj_)) - Pj_};
    }

    /** Print factor details */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorTrapezoidalIntegrator(" << keyFormatter(Factor::keys_[0]) << ","
                  << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2]) << ","
                  << keyFormatter(Factor::keys_[3]) << "," << keyFormatter(Factor::keys_[4]) << ","
                  << keyFormatter(Factor::keys_[5]) << ")\n";
        gtsam::traits<double>::Print(dt_, "  dt: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(e->dt_, dt_, tol);
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
            "FactorTrapezoidalIntegrator", boost::serialization::base_object<Base>(*this));
    }
#endif
};

/**
 * @brief Variant of FactorTrapezoidalIntegrator using Pose3 states
 *
 * This factor is functionally identical to FactorTrapezoidalIntegrator but
 * operates on full 6-DOF poses (gtsam::Pose3) instead of separate position
 * and rotation variables. Position and orientation are automatically extracted
 * from the pose states.
 *
 * **Use when:**
 * - Your state graph uses Pose3 instead of separate Point3 + Rot3
 * - You want fewer state variables (one Pose3 vs two variables)
 *
 * **Example Usage:**
 * @code
 * using namespace mola::factors;
 *
 * // Using Pose3 for state (position + orientation combined)
 * gtsam::Key kTi = gtsam::Symbol('x', 0);  // Pose at time i
 * gtsam::Key kVi = gtsam::Symbol('v', 0);  // Velocity at time i
 * gtsam::Key kTj = gtsam::Symbol('x', 1);  // Pose at time j
 * gtsam::Key kVj = gtsam::Symbol('v', 1);  // Velocity at time j
 *
 * auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
 * double dt = 0.1;
 *
 * graph.add(FactorTrapezoidalIntegratorPose(kTi, kVi, kTj, kVj, dt, noise));
 * @endcode
 *
 * @see FactorTrapezoidalIntegrator for detailed mathematical description
 */
class FactorTrapezoidalIntegratorPose : public gtsam::ExpressionFactorN<
                                            gtsam::Point3 /*return type*/,
                                            gtsam::Pose3,  // Ti
                                            gtsam::Point3,  // bVi
                                            gtsam::Pose3,  // Tj
                                            gtsam::Point3  // bVj
                                            >
{
   private:
    using This = FactorTrapezoidalIntegratorPose;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3 /*return type*/,  //
        gtsam::Pose3, gtsam::Point3,  // Ti, bVi
        gtsam::Pose3, gtsam::Point3  // Tj, bVj
        >;

    double dt_ = .0;  ///< Time interval Δt (seconds)

   public:
    /// Default constructor for serialization
    FactorTrapezoidalIntegratorPose() = default;

    /**
     * @brief Construct a trapezoidal integrator factor using Pose3
     *
     * @param kTi Key for initial pose (contains position and orientation)
     * @param kVi Key for initial velocity (body frame)
     * @param kTj Key for final pose
     * @param kVj Key for final velocity (body frame)
     * @param dt Time interval
     * @param model Noise model (3D position error)
     */
    FactorTrapezoidalIntegratorPose(
        gtsam::Key kTi, gtsam::Key kVi, gtsam::Key kTj, gtsam::Key kVj, const double dt,
        const gtsam::SharedNoiseModel& model)
        : Base({kTi, kVi, kTj, kVj}, model, /* error=0 */ {0, 0, 0}), dt_(dt)
    {
        this->initialize(This::expression({kTi, kVi, kTj, kVj}));
    }

    /// @return Deep copy of this factor
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
     * @brief Measurement expression extracting position/rotation from poses
     *
     * @param keys Array containing [kTi, kVi, kTj, kVj]
     * @return Expression computing position integration error
     */
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Expression<gtsam::Point3> Pi_ = gtsam::translation(gtsam::Pose3_(keys[0]));
        gtsam::Expression<gtsam::Rot3>   Ri_ = gtsam::rotation(gtsam::Pose3_(keys[0]));
        gtsam::Expression<gtsam::Point3> bVi_(keys[1]);

        gtsam::Expression<gtsam::Point3> Pj_ = gtsam::translation(gtsam::Pose3_(keys[2]));
        gtsam::Expression<gtsam::Rot3>   Rj_ = gtsam::rotation(gtsam::Pose3_(keys[2]));
        gtsam::Expression<gtsam::Point3> bVj_(keys[3]);

        return {Pi_ + 0.5 * dt_ * (gtsam::rotate(Ri_, bVi_) + gtsam::rotate(Rj_, bVj_)) - Pj_};
    }

    /** Print factor details */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorTrapezoidalIntegratorPose(" << keyFormatter(Factor::keys_[0])
                  << "," << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2])
                  << "," << keyFormatter(Factor::keys_[3]) << ")\n";
        gtsam::traits<double>::Print(dt_, "  dt: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(e->dt_, dt_, tol);
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
            "FactorTrapezoidalIntegratorPose", boost::serialization::base_object<Base>(*this));
    }
#endif
};

}  // namespace mola::factors