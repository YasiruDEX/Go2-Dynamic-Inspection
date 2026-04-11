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
 * @file   FactorGnssMap2Enu.h
 * @brief  GNSS factor for usage with a T_enu2_map transform symbol
 * @author Jose Luis Blanco Claraco
 * @date   Dec 3, 2025
 */

#pragma once

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{

/**
 * @brief Factor for GNSS measurements with explicit ENU-to-map transform optimization
 *
 * This factor extends FactorGnssEnu by adding a transformation between the local
 * map coordinate system and the ENU coordinate system. This is useful when:
 * - Georeferencing a SLAM map to global coordinates
 * - The initial map origin is arbitrary (e.g., starting pose)
 * - You want to optimize the map's global position and orientation
 *
 * **Mathematical Model:**
 * \f[
 *   p_{antenna}^{ENU} = T_{ENU \leftarrow map} \cdot T_{map \leftarrow i} \oplus
 * p_{sensor}^{vehicle}
 * \f]
 *
 * where:
 * - \f$T_{map \leftarrow i}\f$ is the vehicle pose in the local map frame
 * - \f$T_{ENU \leftarrow map}\f$ is the transform from map to ENU coordinates
 * - This transform is part of the optimization variables
 *
 * **Use Cases:**
 * - SLAM systems that build maps in local coordinates but need global alignment
 * - Loop closure with global constraints
 * - Multi-session SLAM with consistent global frame
 * - Incremental georeferencing as more GNSS data arrives
 *
 * **Example Usage:**
 * @code
 * using namespace mola::factors;
 *
 * // Keys
 * gtsam::Key kEnuToMap = gtsam::Symbol('T', 0);   // ENU-to-map transform (to optimize)
 * gtsam::Key kVehicle  = gtsam::Symbol('x', 10);  // Vehicle pose in map frame
 *
 * // GNSS measurement
 * gtsam::Point3 gnss_enu(1523.4, 2341.7, 15.3);
 * gtsam::Point3 antenna_offset(0.5, 0.0, 1.8);
 * auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(3.0, 3.0, 5.0));
 *
 * // Add factor
 * graph.add(FactorGnssMapEnu(
 *     kEnuToMap,
 *     kVehicle,
 *     antenna_offset,
 *     gnss_enu,
 *     noise
 * ));
 *
 * // Initialize map-to-ENU transform (e.g., identity as prior)
 * values.insert(kEnuToMap, gtsam::Pose3::Identity());
 * @endcode
 *
 * @see FactorGnssEnu for direct pose constraints without transform optimization
 */
class FactorGnssMapEnu
    : public gtsam::ExpressionFactorN<
          gtsam::Point3 /*return*/, gtsam::Pose3 /* T_enu_2_map */, gtsam::Pose3 /* T_map_2_i */
          >
{
   private:
    using This = FactorGnssMapEnu;
    using Base = gtsam::ExpressionFactorN<gtsam::Point3, gtsam::Pose3, gtsam::Pose3>;

    gtsam::Point3 sensorOnVehicle_ = {0, 0, 0};

   public:
    /// Default constructor for serialization
    FactorGnssMapEnu();

    /**
     * @brief Construct a GNSS factor with map transform
     *
     * @param kT_enu2map Key for ENU-to-map coordinate transform
     * @param kT_map2i Key for vehicle pose in map coordinates
     * @param sensorOnVehicle GNSS antenna position in vehicle frame
     * @param observedENU Measured GNSS position in ENU
     * @param model Noise model (3D position uncertainty)
     */
    FactorGnssMapEnu(
        gtsam::Key kT_enu2map, gtsam::Key kT_map2i, const gtsam::Point3& sensorOnVehicle,
        const gtsam::Point3& observedENU, const gtsam::SharedNoiseModel& model)
        : Base({kT_enu2map, kT_map2i}, model, observedENU), sensorOnVehicle_(sensorOnVehicle)
    {
        this->initialize(expression({kT_enu2map, kT_map2i}));
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
     * Computes: transformFrom(T_enu2map * T_map2i, sensorOnVehicle)
     *
     * @param keys Array containing [kT_enu2map, kT_map2i]
     * @return Expression for antenna position in ENU
     */
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Pose3_ T_enu2map(keys[0]);
        gtsam::Pose3_ T_map2i(keys[1]);

        return {gtsam::transformFrom(T_enu2map * T_map2i, gtsam::Point3_(sensorOnVehicle_))};
    }

    /** Print factor details */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorGnssMapEnu(" << keyFormatter(Factor::keys_[0]) << ", "
                  << keyFormatter(Factor::keys_[1]) << ")\n";
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
            "FactorGnssMapEnu", boost::serialization::base_object<Base>(*this));
    }
#endif
};
}  // namespace mola::factors