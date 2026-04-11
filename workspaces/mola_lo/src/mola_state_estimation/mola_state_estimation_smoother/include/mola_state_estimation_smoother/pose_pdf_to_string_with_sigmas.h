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
 * @file   pose_pdf_to_string_with_sigmas.h
 * @brief  Utility to print a 3D pose with 1 sigma interval
 * @author Jose Luis Blanco Claraco
 * @date   Dec 5, 2025
 */

#pragma once

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>

#include <string>

namespace mola::state_estimation_smoother
{

/**
 * @brief Converts a 3D pose Gaussian PDF into a human-readable string.
 *
 * This function extracts the mean pose and the 1-sigma standard deviations
 * (square roots of the covariance diagonal) from an
 * `mrpt::poses::CPose3DPDFGaussian`, and formats them into a multi-line
 * string of the form:
 *
 * ```
 * x = <mean> ± <sigma> [m]
 * y = <mean> ± <sigma> [m]
 * z = <mean> ± <sigma> [m]
 * yaw = <mean> ± <sigma> [deg]
 * pitch = <mean> ± <sigma> [deg]
 * roll = <mean> ± <sigma> [deg]
 * ```
 *
 * Translation components are expressed in meters, and rotational components
 * in degrees.
 *
 * @param pdf The Gaussian PDF representing the 3D pose mean and covariance.
 * @return A `std::string` containing the formatted pose and uncertainties.
 */
std::string pose_pdf_to_string_with_sigmas(const mrpt::poses::CPose3DPDFGaussian& pdf);

/// @overload For poses with covariance inverse
std::string pose_pdf_to_string_with_sigmas(const mrpt::poses::CPose3DPDFGaussianInf& pdf);

}  // namespace mola::state_estimation_smoother
