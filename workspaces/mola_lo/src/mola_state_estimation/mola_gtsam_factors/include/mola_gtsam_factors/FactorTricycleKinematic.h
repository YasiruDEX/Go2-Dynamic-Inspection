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
 * @file   FactorTricycleKinematic.h
 * @brief  GTSAM factor for tricycle/Ackermann kinematic motion model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2026
 *
 * This file implements a kinematic motion model suitable for car-like vehicles
 * with Ackermann steering (front-wheel steering with non-holonomic constraints).
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

#include <cmath>

namespace mola::factors
{

/**
 * @brief Factor implementing tricycle kinematic motion model with Euler integration
 *
 * This factor models the motion of a vehicle with Ackermann steering (like cars,
 * motorcycles, or tricycles) based on linear and angular velocities. The motion
 * follows circular arc trajectories determined by the turn radius.
 *
 * **Kinematic Model:**
 *
 * For angular velocity ω ≠ 0:
 * - The vehicle follows a circular arc with radius R = v/ω
 * - Position change: Δp = (v/ω) * [sin(ω·Δt), 1-cos(ω·Δt), 0]^T (in 2D)
 * - Orientation change: Δθ = ω·Δt
 *
 * For ω ≈ 0 (straight line motion):
 * - Position change: Δp = v·Δt * [1, 0, 0]^T (forward direction)
 * - Orientation change: Δθ ≈ 0
 *
 * The factor uses **Euler integration** (forward integration at initial time):
 * \f[
 *   T_j = T_i \oplus f(v_i, \omega_i, \Delta t)
 * \f]
 *
 * where velocities are measured in the body frame at time i.
 *
 * **State Variables:**
 * - T_i: SE(3) pose at time i (position + orientation)
 * - v_i: Linear velocity vector in body frame [vx, vy, vz] (m/s)
 * - ω_i: Angular velocity vector in body frame [ωx, ωy, ωz] (rad/s)
 * - T_j: SE(3) pose at time j (predicted)
 *
 * **Use Cases:**
 * - Wheel odometry integration for cars, trucks, robots with Ackermann steering
 * - Smoothing trajectories of wheeled vehicles
 * - Fusing encoder measurements with other sensors
 * - Ground vehicle state estimation
 *
 * **Example Usage:**
 * @code
 * using namespace mola::factors;
 *
 * gtsam::NonlinearFactorGraph graph;
 *
 * // State variables
 * gtsam::Key kT0 = gtsam::Symbol('x', 0);  // Initial pose
 * gtsam::Key kV0 = gtsam::Symbol('v', 0);  // Linear velocity
 * gtsam::Key kW0 = gtsam::Symbol('w', 0);  // Angular velocity
 * gtsam::Key kT1 = gtsam::Symbol('x', 1);  // Final pose
 *
 * // Noise model (6D: 3D rotation + 3D translation error)
 * auto noise = gtsam::noiseModel::Diagonal::Sigmas(
 *     (gtsam::Vector(6) << 0.01, 0.01, 0.01,  // rotation std dev (rad)
 *                          0.05, 0.05, 0.02   // translation std dev (m)
 *     ).finished()
 * );
 *
 * double dt = 0.1; // 100ms time step
 *
 * // Add kinematic constraint
 * graph.add(FactorTricycleKinematic(kT0, kV0, kW0, kT1, dt, noise));
 * @endcode
 *
 * **Notes:**
 * - For 2D motion (ground vehicles), vz and ωx, ωy should typically be zero
 * - The threshold parameter controls when ω is considered "close to zero"
 * - Analytical Jacobians are provided
 *
 * @see FactorAngularVelocityIntegration for rotation-only integration
 * @see FactorTrapezoidalIntegrator for trapezoidal integration (uses both v_i and v_j)
 */
class FactorTricycleKinematic : public gtsam::NoiseModelFactor4<
                                    gtsam::Pose3, gtsam::Point3, gtsam::Point3,  // Ti, bVi, bWi
                                    gtsam::Pose3>  // Tj
{
   private:
    using This = FactorTricycleKinematic;
    using Base = gtsam::NoiseModelFactor4<
        gtsam::Pose3, gtsam::Point3, gtsam::Point3,  // Ti, bVi, bWi
        gtsam::Pose3>;  // Tj

    double dt_          = 0.0;  ///< Time interval Δt (seconds)
    double w_threshold_ = 1e-4;  ///< Threshold for considering ω ≈ 0 (rad/s)

   public:
    /// Default constructor for serialization
    FactorTricycleKinematic();

    /**
     * @brief Construct a tricycle kinematic factor
     *
     * @param kTi Key for initial pose T_i
     * @param kVi Key for initial linear velocity v_i in body frame (m/s)
     * @param kWi Key for initial angular velocity ω_i in body frame (rad/s)
     * @param kTj Key for final pose T_j
     * @param dt Time interval Δt (seconds)
     * @param model Noise model (6D: rotation + translation error)
     * @param w_threshold Threshold below which ω is treated as zero (default: 1e-4 rad/s)
     *                    Adjust if numerical issues occur near zero angular velocity
     */
    FactorTricycleKinematic(
        gtsam::Key kTi, gtsam::Key kVi, gtsam::Key kWi, gtsam::Key kTj, const double dt,
        const gtsam::SharedNoiseModel& model, const double w_threshold = 1e-4);

    /// @return Deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    /**
     * @brief Compute predicted pose using tricycle kinematic model
     *
     * Static helper function that can be used independently for forward prediction.
     *
     * @param Ti Initial pose
     * @param bVi Linear velocity in body frame
     * @param bWi Angular velocity in body frame
     * @param dt Time step
     * @param w_threshold Threshold for straight-line motion
     * @return Predicted pose T_j
     */
    static gtsam::Pose3 tricycleKinematicModel(
        const gtsam::Pose3& Ti, const gtsam::Point3& bVi, const gtsam::Point3& bWi, double dt,
        double w_threshold);

    /**
     * @brief Evaluate the error of this factor
     *
     * Computes the relative pose error between the predicted pose (from kinematics)
     * and the actual T_j value.
     *
     * @param Ti Initial pose
     * @param bVi Linear velocity
     * @param bWi Angular velocity
     * @param Tj Actual final pose
     * @param H1 Optional Jacobian w.r.t. Ti
     * @param H2 Optional Jacobian w.r.t. bVi
     * @param H3 Optional Jacobian w.r.t. bWi
     * @param H4 Optional Jacobian w.r.t. Tj
     * @return 6D error vector [rotation_error(3D), translation_error(3D)]
     */
    gtsam::Vector evaluateError(
        const gtsam::Pose3& Ti, const gtsam::Point3& bVi, const gtsam::Point3& bWi,
        const gtsam::Pose3& Tj,
#if GTSAM_USES_BOOST
        boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
        boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4
#else
        gtsam::Matrix* H1 = nullptr, gtsam::Matrix* H2 = nullptr, gtsam::Matrix* H3 = nullptr,
        gtsam::Matrix* H4 = nullptr
#endif
    ) const override;

    /** Print factor information */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorTricycleKinematic(" << keyFormatter(this->keys_[0]) << ","
                  << keyFormatter(this->keys_[1]) << "," << keyFormatter(this->keys_[2]) << ","
                  << keyFormatter(this->keys_[3]) << ")\n";
        gtsam::traits<double>::Print(dt_, "  dt: ");
        gtsam::traits<double>::Print(w_threshold_, "  w_threshold: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(e->dt_, dt_, tol) &&
               gtsam::traits<double>::Equals(e->w_threshold_, w_threshold_, tol);
    }

   private:
#if GTSAM_USES_BOOST
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& boost::serialization::make_nvp(
            "FactorTricycleKinematic", boost::serialization::base_object<Base>(*this));
        ar& BOOST_SERIALIZATION_NVP(dt_);
        ar& BOOST_SERIALIZATION_NVP(w_threshold_);
    }
#endif
};

}  // namespace mola::factors