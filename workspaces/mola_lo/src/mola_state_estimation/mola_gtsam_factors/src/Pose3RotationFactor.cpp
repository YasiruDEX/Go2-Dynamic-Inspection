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
 * @file   Pose3RotationFactor.cpp
 * @brief  GTSAM factor for observations on the Rot3 part only of a Pose3
 * @author Jose Luis Blanco Claraco
 * @date   Dec 9, 2025
 */

#include <mola_gtsam_factors/Pose3RotationFactor.h>

namespace mola::factors
{

Pose3RotationFactor::Pose3RotationFactor() = default;

Pose3RotationFactor::Pose3RotationFactor(
    gtsam::Key kT_enu2map, gtsam::Key kPi, const gtsam::Pose3& sensorPoseOnVehicle,
    const gtsam::Rot3& observedOrientation, const gtsam::SharedNoiseModel& model)
    : Base({kT_enu2map, kPi}, model, /* error */ observedOrientation),
      sensorPoseOnVehicle_(sensorPoseOnVehicle)
{
    this->initialize(This::expression({kT_enu2map, kPi}));
}

gtsam::NonlinearFactor::shared_ptr Pose3RotationFactor::clone() const
{
#if GTSAM_USES_BOOST
    return boost::static_pointer_cast<This>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
    return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
}

gtsam::Expression<gtsam::Rot3> Pose3RotationFactor::expression(
    const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const
{
    const gtsam::Expression<gtsam::Pose3> T_enu2map_(keys[0]);
    const gtsam::Expression<gtsam::Pose3> Pi_(keys[1]);
    const gtsam::Expression<gtsam::Pose3> p(sensorPoseOnVehicle_);

    return {gtsam::rotation(gtsam::compose(T_enu2map_, gtsam::compose(Pi_, p)))};
}

void Pose3RotationFactor::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const
{
    std::cout << s << "Pose3RotationFactor(" << keyFormatter(Factor::keys_[0]) << ", "
              << keyFormatter(Factor::keys_[1]) << ")\n";
    gtsam::traits<gtsam::Rot3>::Print(measured_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
}

bool Pose3RotationFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const
{
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol);
}

}  // namespace mola::factors