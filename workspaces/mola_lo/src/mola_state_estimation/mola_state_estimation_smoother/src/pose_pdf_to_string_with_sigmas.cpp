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
 * @file   pose_pdf_to_string_with_sigmas.cpp
 * @brief  Utility to print a 3D pose with 1 sigma interval
 * @author Jose Luis Blanco Claraco
 * @date   Dec 5, 2025
 */

#include <mola_state_estimation_smoother/pose_pdf_to_string_with_sigmas.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace
{

std::string impl_pose_pdf_to_str(
    const mrpt::poses::CPose3D& m, const double sx, const double sy, const double sz,
    const double sigma_yaw, const double sigma_pitch, const double sigma_roll)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    oss << "x = " << m.x() << " ± " << sx << " [m]\n";
    oss << "y = " << m.y() << " ± " << sy << " [m]\n";
    oss << "z = " << m.z() << " ± " << sz << " [m]\n";
    oss << "yaw = " << mrpt::RAD2DEG(m.yaw()) << " ± " << sigma_yaw << " [deg]\n";
    oss << "pitch = " << mrpt::RAD2DEG(m.pitch()) << " ± " << sigma_pitch << " [deg]\n";
    oss << "roll = " << mrpt::RAD2DEG(m.roll()) << " ± " << sigma_roll << " [deg]\n";

    return oss.str();
}

}  // namespace

std::string mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(
    const mrpt::poses::CPose3DPDFGaussianInf& pdf)
{
    const auto& m = pdf.mean;  // mrpt::poses::CPose3D
    const auto& C = pdf.cov_inv;  // mrpt::math::CMatrixDouble66

    // 1-sigma std deviations = sqrt(variance)
    const double sx          = std::sqrt(1.0 / C(0, 0));
    const double sy          = std::sqrt(1.0 / C(1, 1));
    const double sz          = std::sqrt(1.0 / C(2, 2));
    const double sigma_yaw   = mrpt::RAD2DEG(1.0 / std::sqrt(C(3, 3)));
    const double sigma_pitch = mrpt::RAD2DEG(1.0 / std::sqrt(C(4, 4)));
    const double sigma_roll  = mrpt::RAD2DEG(1.0 / std::sqrt(C(5, 5)));

    return impl_pose_pdf_to_str(m, sx, sy, sz, sigma_yaw, sigma_pitch, sigma_roll);
}

std::string mola::state_estimation_smoother::pose_pdf_to_string_with_sigmas(
    const mrpt::poses::CPose3DPDFGaussian& pdf)
{
    const auto& m = pdf.mean;  // mrpt::poses::CPose3D
    const auto& C = pdf.cov;  // mrpt::math::CMatrixDouble66

    // 1-sigma std deviations = sqrt(variance)
    const double sx          = std::sqrt(C(0, 0));
    const double sy          = std::sqrt(C(1, 1));
    const double sz          = std::sqrt(C(2, 2));
    const double sigma_yaw   = mrpt::RAD2DEG(std::sqrt(C(3, 3)));
    const double sigma_pitch = mrpt::RAD2DEG(std::sqrt(C(4, 4)));
    const double sigma_roll  = mrpt::RAD2DEG(std::sqrt(C(5, 5)));

    return impl_pose_pdf_to_str(m, sx, sy, sz, sigma_yaw, sigma_pitch, sigma_roll);
}
