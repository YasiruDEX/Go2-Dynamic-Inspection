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

// MOLA+MRPT
#include <mola_georeferencing/simplemap_georeference.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

#if __has_include(<mp2p_icp/update_velocity_buffer_from_obs.h>)
#include <mola_imu_preintegration/LocalVelocityBuffer.h>
#include <mp2p_icp/update_velocity_buffer_from_obs.h>
#define HAS_VELOCITY_BUFFER
#endif

// gtsam factors:
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/FactorGnssEnu.h>
#include <mola_gtsam_factors/MeasuredGravityFactor.h>

mola::SMGeoReferencingOutput mola::simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm, const SMGeoReferencingParams& params)
{
    mola::SMGeoReferencingOutput ret;

    ASSERT_(!sm.empty());

    const GNSSFrames smFrames = extract_gnss_frames_from_sm(sm, params.geodeticReference);

    if (params.logger)
    {
        std::stringstream ss;
        ss << "[simplemap_georeference] Found: " << smFrames.frames.size() << " GNSS frames";
        params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
    }

    // we check GNSS frames later on, to check if we have at least IMU data.

    // Build and optimize GTSAM graph:
    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               v;

    add_gnss_factors(graph, v, smFrames, params.fgParams);

    // Collect which P(kf_index) keys are already in the graph from GNSS:
    std::set<size_t> existingPoseKeys;
    for (const auto& f : smFrames.frames)
    {
        existingPoseKeys.insert(f.kf_index);
    }

    bool hasIMUGravityFactors = false;
    if (params.useIMUGravityAlignment)
    {
        const IMUAccFrames imuFrames = extract_imu_acc_frames_from_sm(sm);

        if (!imuFrames.frames.empty())
        {
            hasIMUGravityFactors = true;
        }

        if (params.logger)
        {
            std::stringstream ss;
            ss << "[simplemap_georeference] Found: " << imuFrames.frames.size()
               << " IMU acceleration frames";
            params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
        }

        add_imu_gravity_factors(graph, v, imuFrames, existingPoseKeys, params.imuGravityParams);
    }

    if (smFrames.frames.empty() && !hasIMUGravityFactors)
    {
        if (params.logger)
        {
            params.logger->logStr(
                mrpt::system::LVL_ERROR,
                "The input simplemap seems not to have neither GNSS nor IMU acceleration data, so "
                "no georeferencing/gravity alignment can be performed.");
        }
        return ret;
    }

    thread_local bool DEBUG_PRINT_GRAPH =
        mrpt::get_env<bool>("MOLA_SM_GEOREF_PRINT_FACTOR_GRAPH", false);
    if (DEBUG_PRINT_GRAPH)
    {
        graph.print("\n====\nGTSAM graph:\n");
        v.print("\n====\nGTSAM initial values:\n");
    }

    gtsam::LevenbergMarquardtParams lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();

    gtsam::LevenbergMarquardtOptimizer lm(graph, v, lmParams);

    auto optimal = lm.optimize();

    thread_local bool DEBUG_PRINT_FG_ERRORS =
        mrpt::get_env<bool>("MOLA_SM_GEOREF_PRINT_FG_ERRORS", false);
    if (DEBUG_PRINT_FG_ERRORS)
    {
        graph.printErrors(optimal, "\n===\nFG errors:\n");
    }

    const double errInit = graph.error(v);
    const double errEnd  = graph.error(optimal);

    const double rmseInit = std::sqrt(errInit / static_cast<double>(graph.size()));
    const double rmseEnd  = std::sqrt(errEnd / static_cast<double>(graph.size()));

    gtsam::Marginals marginals(graph, optimal);

    const auto T0     = optimal.at<gtsam::Pose3>(T(0));
    const auto T0_cov = marginals.marginalCovariance(T(0));
    const auto stds   = T0_cov.diagonal().array().sqrt().eval();

    if (params.logger)
    {
        std::stringstream ss;
        ss << "[simplemap_georeference] LM iterations: " << lm.iterations()
           << ", init error: " << errInit << " (rmse=" << rmseInit << "), final error: " << errEnd
           << "(rmse=" << rmseEnd << ") , for " << smFrames.frames.size()
           << " frames, GTSAM sigmas: " << mrpt::RAD2DEG(stds[0]) << " [deg], "
           << mrpt::RAD2DEG(stds[1]) << " [deg], " << mrpt::RAD2DEG(stds[2]) << " [deg], "
           << stds[3] << " [m], " << stds[4] << " [m], " << stds[5] << " [m]";
        params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
    }

    // store results:
    ret.geo_ref.emplace();

    // We will always have this T, using IMU or GNSS:
    ret.geo_ref->T_enu_to_map = {
        mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T0)),
        mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T0_cov)};

    // We may not have geodetics reference if using IMU only. Leave lat=lon=h=0
    if (smFrames.refCoord.has_value())
    {
        ret.geo_ref->geo_coord = *smFrames.refCoord;
    }

    ret.final_rmse = rmseEnd;

    return ret;
}

mola::GNSSFrames mola::extract_gnss_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoordIn)
{
    GNSSFrames ret;

    ret.refCoord = refCoordIn;

    ret.frames.reserve(sm.size());

    // Build list of KF poses with GNSS observations:
    for (size_t kfIdx = 0; kfIdx < sm.size(); kfIdx++)
    {
        const auto& [pose, sf, twist] = sm.get(kfIdx);

        ASSERT_(pose);
        ASSERT_(sf);

        const auto p = pose->getMeanVal();

        mrpt::obs::CObservationGPS::Ptr obs;
        for (size_t i = 0; !!(obs = sf->getObservationByClass<mrpt::obs::CObservationGPS>(i)); i++)
        {
            if (!obs->hasMsgType(mrpt::obs::gnss::NMEA_GGA))
            {
                continue;
            }

            auto& f = ret.frames.emplace_back();

            f.pose     = p;
            f.kf_index = kfIdx;
            f.obs      = obs;
            f.gga      = obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

            if (obs->covariance_enu)
            {
                f.sigma_E = std::sqrt((*obs->covariance_enu)(0, 0));
                f.sigma_N = std::sqrt((*obs->covariance_enu)(1, 1));
                f.sigma_U = std::sqrt((*obs->covariance_enu)(2, 2));
            }
            else
            {
                f.sigma_E = f.gga.fields.HDOP * 4.5 /*HDOP_REFERENCE_METERS*/;
                f.sigma_N = f.sigma_E;
                f.sigma_U = f.sigma_E;
            }

            if (f.sigma_E <= 0 || f.sigma_N <= 0 || f.sigma_U <= 0 || std::isnan(f.sigma_E) ||
                std::isnan(f.sigma_N) || std::isnan(f.sigma_U))
            {
                ret.frames.pop_back();  // Remove invalid entry
                continue;  // skip invalid entry
            }

            f.coords.lat    = f.gga.fields.latitude_degrees;
            f.coords.lon    = f.gga.fields.longitude_degrees;
            f.coords.height = f.gga.fields.altitude_meters;

            // keep first one:
            if (!ret.refCoord.has_value())
            {
                ret.refCoord = f.coords;
            }

            // Convert GNSS obs to ENU:
            mrpt::topography::geodeticToENU_WGS84(f.coords, f.enu, *ret.refCoord);
        }
    }

    return ret;
}

void mola::add_gnss_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const GNSSFrames& frames,
    const AddGNSSFactorParams& params)
{
    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose, in the {map} frame
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation: {enu} -> {map}

    v.insert(T(0), gtsam::Pose3::Identity());

    // Expression to optimize (i=0...N):
    // P (+) kf_pose{i} = gps_enu{i}

    auto noisePoses         = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    auto noiseHorizontality = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1e3, 1e3, 1e3, 1e6, 1e6, params.horizontalitySigmaZ));

    for (size_t i = 0; i < frames.frames.size(); i++)
    {
        const auto& frame = frames.frames.at(i);

        auto noiseOrg = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(frame.sigma_E, frame.sigma_N, frame.sigma_U)
                .array()
                .max(params.minimumUncertaintyXYZ));

        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(frame.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(frame.obs->sensorPose.translation());

        fg.emplace_shared<mola::factors::FactorGnssEnu>(
            P(frame.kf_index), sensorPointOnVeh, observedENU, robustNoise);

        const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.pose);

        if (!v.exists(P(frame.kf_index)))
        {
            v.insert(P(frame.kf_index), vehiclePose);
        }

        fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            T(0), P(frame.kf_index), vehiclePose, noisePoses);

        if (params.addHorizontalityConstraints)
        {
            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                P(frame.kf_index), gtsam::Pose3::Identity(), noiseHorizontality);
        }
    }
}

namespace
{
bool imu_acceleration_seems_valid(const gtsam::Vector3& measuredGravity)
{
    const double norm = measuredGravity.norm();

    // Accept both m/s² (~9.8) and already-normalized (~1.0) IMU outputs:
    if (std::abs(norm - 9.8) > 2.0 && std::abs(norm - 1.0) > 0.2)
    {
        return false;
    }

    return true;
}

}  // namespace

mola::IMUAccFrames mola::extract_imu_acc_frames_from_sm(const mrpt::maps::CSimpleMap& sm)
{
    IMUAccFrames ret;
    ret.frames.reserve(sm.size());

    const auto addMeasurement = [&ret](
                                    size_t kfIdx, const mrpt::poses::CPose3D& p,
                                    const mrpt::poses::CPose3D& sensorPose,
                                    const gtsam::Vector3&       measuredGravity)
    {
        if (!imu_acceleration_seems_valid(measuredGravity))
        {
            return;  // skip: not a valid gravity-like reading
        }

        auto& f               = ret.frames.emplace_back();
        f.kf_index            = kfIdx;
        f.vehiclePose         = p;
        f.sensorPoseOnVehicle = sensorPose;
        f.normalizedAcc       = measuredGravity.normalized();
    };

    for (size_t kfIdx = 0; kfIdx < sm.size(); kfIdx++)
    {
        const auto& [pose, sf, twist] = sm.get(kfIdx);

        ASSERT_(pose);
        ASSERT_(sf);

        const auto p = pose->getMeanVal();

        // 1) Process direct CObservationIMU, if available:
        mrpt::obs::CObservationIMU::Ptr obs;
        for (size_t i = 0; !!(obs = sf->getObservationByClass<mrpt::obs::CObservationIMU>(i)); i++)
        {
            if (!obs->has(mrpt::obs::IMU_X_ACC) || !obs->has(mrpt::obs::IMU_Y_ACC) ||
                !obs->has(mrpt::obs::IMU_Z_ACC))
            {
                continue;
            }

            const gtsam::Vector3 measuredGravity = {
                obs->get(mrpt::obs::IMU_X_ACC), obs->get(mrpt::obs::IMU_Y_ACC),
                obs->get(mrpt::obs::IMU_Z_ACC)};

            addMeasurement(kfIdx, p, obs->sensorPose, measuredGravity);
        }

        // 2) Process embedded IMU info embedded into the metadata:
#if defined(HAS_VELOCITY_BUFFER)
        mola::imu::LocalVelocityBuffer lvb;
        for (const auto& o : *sf)
        {
            mp2p_icp::update_velocity_buffer_from_obs(lvb, o);
        }

        // Get the current linear accelerations map (in the vehicle frame of reference)
        // Average them all so there are not too many factors:
        gtsam::Vector3 avr_acc   = gtsam::Vector3::Zero();
        size_t         avr_count = 0;

        for (const auto& [t, measuredGravity] : lvb.get_linear_accelerations())
        {
            const auto acc = mrpt::gtsam_wrappers::toPoint3(measuredGravity);
            if (imu_acceleration_seems_valid(acc))
            {
                avr_count++;
                avr_acc += acc;
            }
        }

        if (avr_count > 0)
        {
            addMeasurement(
                kfIdx, p, mrpt::poses::CPose3D::Identity(),
                avr_acc / static_cast<double>(avr_count));
        }

#endif
    }  // end for each SM keyframe

    return ret;
}

void mola::add_imu_gravity_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const IMUAccFrames& imuFrames,
    const std::set<size_t>& existingPoseKeys, const AddIMUGravityFactorParams& params)
{
    using gtsam::symbol_shorthand::P;
    using gtsam::symbol_shorthand::T;

    auto noisePoses = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    auto accNoise =
        gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(params.imuGravitySigmaDeg));

    // If T(0) was not added because there were no GNSS frames, add it now:
    if (existingPoseKeys.empty())
    {
        if (!v.exists(T(0)))
        {
            v.insert(T(0), gtsam::Pose3::Identity());
        }

        // Also, add a weak prior to anchor the undefined azimuth angle, which is unobservable
        // without GNSS:

        //  Used when there are IMU factors but no GNSS data, to "anchor" the solution in the
        //  azimuth angle, which is unobservable with gravity-only factors [rad] */
        double azimuthUnobservableSigma = 1.0;

        auto noisePriorT0 = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector6(1.0, 1.0, azimuthUnobservableSigma, 1.0, 1.0, 1.0));
        fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(0), gtsam::Pose3::Identity(), noisePriorT0);
    }

    for (const auto& frame : imuFrames.frames)
    {
        const auto key = P(frame.kf_index);

        // If this KF doesn't already have a P variable (from GNSS), create one:
        if (existingPoseKeys.count(frame.kf_index) == 0 && !v.exists(key))
        {
            const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.vehiclePose);
            v.insert(key, vehiclePose);

            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key, vehiclePose, noisePoses);
        }

        const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(frame.sensorPoseOnVehicle);

        fg.emplace_shared<mola::factors::MeasuredGravityFactor>(
            T(0), key, sensorOnVehicle, frame.normalizedAcc, accNoise);
    }
}
