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

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/system/COutputLogger.h>

// GTSAM:
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace mola
{
struct SMGeoReferencingOutput
{
    SMGeoReferencingOutput() = default;

    /// Will be nullopt if georeferencing failed, e.g. due to insufficient GNSS
    /// data.
    std::optional<mp2p_icp::metric_map_t::Georeferencing> geo_ref;

    double final_rmse = .0;
};

struct AddGNSSFactorParams
{
    /// May be required for small maps, i.e. when the length of the trajectory
    /// is not >10 times the GNSS uncertainty.
    bool   addHorizontalityConstraints = false;
    double horizontalitySigmaZ         = 1.0;  // [m]

    double minimumUncertaintyXYZ = 0.20;  // [m]
};

struct AddIMUGravityFactorParams
{
    double imuGravitySigmaDeg = 3.0;  // [deg]
};

struct SMGeoReferencingParams
{
    SMGeoReferencingParams() = default;

    /// If provided, this will be the coordinates of the ENU frame origin.
    /// Otherwise (default), the first GNSS entry will become the reference.
    std::optional<mrpt::topography::TGeodeticCoords> geodeticReference;

    AddGNSSFactorParams fgParams;

    /// If true, use IMU acceleration data found in the simplemap to add
    /// gravity-alignment constraints (MeasuredGravityFactor) to the
    /// optimization. This helps in cases with poor Z data in GPS.
    bool                      useIMUGravityAlignment = true;
    AddIMUGravityFactorParams imuGravityParams;

    mrpt::system::COutputLogger* logger = nullptr;
};

/** Function to georeferencing a given SimpleMap with GNSS observations.
 *  A minimum of 3 (non-colinear) KFs with GNSS data are required to solve
 *  for the optimal transformation.
 */
SMGeoReferencingOutput simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm, const SMGeoReferencingParams& params = {});

struct FrameGNSS
{
    mrpt::poses::CPose3D              pose;
    size_t                            kf_index = 0;  //!< Index in the source CSimpleMap
    mrpt::obs::CObservationGPS::Ptr   obs;
    mrpt::obs::gnss::Message_NMEA_GGA gga;
    mrpt::topography::TGeodeticCoords coords;
    mrpt::math::TPoint3D              enu;
    double                            sigma_E = 5.0;
    double                            sigma_N = 5.0;
    double                            sigma_U = 5.0;
};

struct GNSSFrames
{
    std::vector<FrameGNSS>                           frames;
    std::optional<mrpt::topography::TGeodeticCoords> refCoord;
};

GNSSFrames extract_gnss_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoord = std::nullopt);

struct FrameIMUAcc
{
    size_t               kf_index = 0;  //!< Index in the source CSimpleMap
    mrpt::poses::CPose3D vehiclePose;
    mrpt::poses::CPose3D sensorPoseOnVehicle;
    gtsam::Vector3       normalizedAcc;
};

struct IMUAccFrames
{
    std::vector<FrameIMUAcc> frames;
};

IMUAccFrames extract_imu_acc_frames_from_sm(const mrpt::maps::CSimpleMap& sm);

/// Adds MeasuredGravityFactor factors to the graph. Creates new P(kf_index)
/// variables for keyframes not already present in \a existingPoseKeys.
void add_imu_gravity_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const IMUAccFrames& imuFrames,
    const std::set<size_t>& existingPoseKeys, const AddIMUGravityFactorParams& params);

void add_gnss_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const GNSSFrames& frames,
    const AddGNSSFactorParams& params);

}  // namespace mola
