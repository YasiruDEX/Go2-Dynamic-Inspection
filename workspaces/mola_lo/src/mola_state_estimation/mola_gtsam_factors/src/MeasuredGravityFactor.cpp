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
 * @file   MeasuredGravityFactor.cpp
 * @brief  GTSAM factor for IMU observations of gravity-aligned orientation.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 9, 2025
 */

#include <mola_gtsam_factors/MeasuredGravityFactor.h>

namespace mola::factors
{

MeasuredGravityFactor::MeasuredGravityFactor() = default;

MeasuredGravityFactor::MeasuredGravityFactor(
    gtsam::Key kT_enu2map, gtsam::Key kPi, const gtsam::Pose3& sensorPoseOnVehicle,
    const gtsam::Vector3& observedNormalizedAcc_b, const gtsam::SharedNoiseModel& model)
    : Base({kT_enu2map, kPi}, model, /* error */ {.0, .0, 1.}),
      sensorPoseOnVehicle_(sensorPoseOnVehicle),
      observedNormalizedAcc_b_(observedNormalizedAcc_b)
{
    this->initialize(This::expression({kT_enu2map, kPi}));
}

gtsam::NonlinearFactor::shared_ptr MeasuredGravityFactor::clone() const
{
#if GTSAM_USES_BOOST
    return boost::static_pointer_cast<This>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
    return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
}

gtsam::Expression<gtsam::Point3> MeasuredGravityFactor::expression(
    const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const
{
    const gtsam::Expression<gtsam::Pose3> T_enu2map_(keys[0]);
    const gtsam::Expression<gtsam::Pose3> Pi_(keys[1]);
    const gtsam::Expression<gtsam::Pose3> p(sensorPoseOnVehicle_);

    const gtsam::Expression<gtsam::Point3> measuredUp(observedNormalizedAcc_b_);

    return {gtsam::rotate(
        gtsam::rotation(gtsam::compose(T_enu2map_, gtsam::compose(Pi_, p))), measuredUp)};
}

void MeasuredGravityFactor::print(
    const std::string& s, const gtsam::KeyFormatter& keyFormatter) const
{
    std::cout << s << "MeasuredGravityFactor(" << keyFormatter(Factor::keys_[0]) << ", "
              << keyFormatter(Factor::keys_[1]) << ")\n";
    gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
}

bool MeasuredGravityFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const
{
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol);
}

}  // namespace mola::factors