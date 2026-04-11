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

/**
 * @file   FactorGnssEnu.h
 * @brief  GTSAM factor for GNSS/GPS measurements in ENU coordinates
 * @author Jose Luis Blanco Claraco
 * @date   Dec 3, 2025
 *
 * This file provides factors for incorporating GNSS (Global Navigation Satellite System)
 * measurements into factor graphs for localization and mapping.
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{

/**
 * @brief Factor for GNSS position measurements in East-North-Up (ENU) coordinates
 *
 * This factor constrains a vehicle pose based on GNSS position observations.
 * The GNSS antenna position is computed by transforming the sensor offset from
 * the vehicle frame to the global ENU frame.
 *
 * **Coordinate Systems:**
 * - **ENU (East-North-Up)**: Local tangent plane coordinate system
 *   - Origin: arbitrary reference point (e.g., first GNSS fix)
 *   - X-axis: East direction
 *   - Y-axis: North direction
 *   - Z-axis: Up (perpendicular to Earth ellipsoid)
 *
 * **Mathematical Model:**
 * \f[
 *   p_{antenna}^{ENU} = T_i \oplus p_{sensor}^{vehicle}
 * \f]
 *
 * where:
 * - \f$T_i\f$ is the vehicle pose in ENU frame
 * - \f$p_{sensor}^{vehicle}\f$ is the antenna offset in vehicle coordinates
 * - \f$p_{antenna}^{ENU}\f$ is the measured GNSS position
 *
 * **Use Cases:**
 * - GPS/GNSS integration in SLAM systems
 * - Georeferencing local maps
 * - Drift correction for odometry-based navigation
 * - Multi-sensor fusion with IMU, wheel encoders, etc.
 *
 * **Example Usage:**
 * @code
 * using namespace mola::factors;
 *
 * // GNSS measurement in ENU coordinates (meters)
 * gtsam::Point3 gnss_enu(1523.4, 2341.7, 15.3);  // East, North, Up
 *
 * // Antenna offset from vehicle center (meters, in vehicle frame)
 * gtsam::Point3 antenna_offset(0.5, 0.0, 1.8);  // 0.5m forward, 1.8m up
 *
 * // Measurement uncertainty (typically 2-10m horizontal, 5-20m vertical for civilian GPS)
 * auto noise = gtsam::noiseModel::Diagonal::Sigmas(
 *     gtsam::Vector3(3.0, 3.0, 5.0)  // σ_east, σ_north, σ_up in meters
 * );
 *
 * // Add GNSS factor
 * gtsam::Key vehicle_pose_key = gtsam::Symbol('x', 10);
 * graph.add(FactorGnssEnu(
 *     vehicle_pose_key,
 *     antenna_offset,
 *     gnss_enu,
 *     noise
 * ));
 * @endcode
 *
 * **Notes:**
 * - GNSS measurements should be converted from Lat/Lon/Alt to ENU before use
 * - Noise model should reflect actual GNSS accuracy (varies with satellite visibility,
 *   multipath, atmospheric conditions, receiver quality, etc.)
 * - For RTK-GPS, use much smaller noise values (cm-level accuracy)
 * - Consider using FactorGnssMapEnu if optimizing map-to-ENU alignment
 *
 * @see FactorGnssMapEnu for georeferencing with explicit map transform
 */
class FactorGnssEnu : public gtsam::ExpressionFactorN<
                          gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/
                          >
{
   private:
    using This = FactorGnssEnu;
    using Base = gtsam::ExpressionFactorN<gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/>;

    gtsam::Point3 sensorOnVehicle_ = {0, 0, 0};  ///< Antenna offset in vehicle frame

   public:
    /// Default constructor for serialization
    FactorGnssEnu();

    /**
     * @brief Construct a GNSS ENU factor
     *
     * @param kPi Key for vehicle pose in ENU frame
     * @param sensorOnVehicle GNSS antenna position in vehicle coordinates (meters)
     *                        For example: (0.5, 0, 1.8) means 0.5m forward, 1.8m above vehicle
     * origin
     * @param observedENU Measured GNSS position in ENU coordinates (meters)
     * @param model Noise model (3D position uncertainty)
     */
    FactorGnssEnu(
        gtsam::Key kPi, const gtsam::Point3& sensorOnVehicle, const gtsam::Point3& observedENU,
        const gtsam::SharedNoiseModel& model)
        : Base({kPi}, model, observedENU), sensorOnVehicle_(sensorOnVehicle)
    {
        this->initialize(expression({kPi}));
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
     * Computes: transformFrom(Pi, sensorOnVehicle)
     *
     * @param keys Array containing vehicle pose key
     * @return Expression for antenna position in ENU
     */
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Pose3_ Pi(keys[0]);

        return {gtsam::transformFrom(Pi, gtsam::Point3_(sensorOnVehicle_))};
    }

    /** Print factor details */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorGnssEnu(" << keyFormatter(Factor::keys_[0]) << ")\n";
        gtsam::traits<gtsam::Point3>::Print(sensorOnVehicle_, "  sensorOnVehicle: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<gtsam::Point3>::Equals(e->sensorOnVehicle_, sensorOnVehicle_, tol);
    }

   private:
#if GTSAM_USES_BOOST
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& BOOST_SERIALIZATION_NVP(measured_);  // params before base class
        ar& BOOST_SERIALIZATION_NVP(sensorOnVehicle_);
        ar& boost::serialization::make_nvp(
            "FactorGnssEnu", boost::serialization::base_object<Base>(*this));
    }
#endif
};
}  // namespace mola::factors