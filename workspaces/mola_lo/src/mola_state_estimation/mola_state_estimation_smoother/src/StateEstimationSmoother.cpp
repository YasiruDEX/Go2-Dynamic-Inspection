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
 * @file   StateEstimationSmoother.cpp
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

// MOLA & MRPT:
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// Custom factors:
#include <mola_gtsam_factors/FactorAngularVelocityIntegration.h>
#include <mola_gtsam_factors/FactorConstLocalVelocity.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_gtsam_factors/FactorTrapezoidalIntegrator.h>
#include <mola_gtsam_factors/FactorTricycleKinematic.h>
#include <mola_gtsam_factors/MeasuredGravityFactor.h>
#include <mola_gtsam_factors/Pose3RotationFactor.h>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(
    StateEstimationSmoother, mola::ExecutableBase, mola::state_estimation_smoother)

namespace
{
constexpr double ENU2MAP_WEAK_SIGMA          = 1e4;
constexpr double INIT_ODOM_FRAME_POSE_SIGMA  = 1e3;
constexpr double FIRST_POSE_WEAK_PRIOR_SIGMA = 1e6;
constexpr double PLANAR_XY_SIGMA             = 1e10;
constexpr double PLANAR_Z_SIGMA              = 1e-4;
constexpr double TRICYCLE_LARGE_SIGMAS       = 1e6;

void enforce_planar_pose(mrpt::poses::CPose3D& p)
{
    p.z(0);
    p.setYawPitchRoll(p.yaw(), .0, .0);
}
void enforce_planar_twist(mrpt::math::TTwist3D& tw)
{
    tw.vz = 0;
    tw.wx = 0;
    tw.wy = 0;
}

}  // namespace

namespace mola::state_estimation_smoother
{

const bool   NAVSTATE_PRINT_FG        = mrpt::get_env<bool>("NAVSTATE_PRINT_FG", false);
const bool   NAVSTATE_PRINT_FG_ERRORS = mrpt::get_env<bool>("NAVSTATE_PRINT_FG_ERRORS", false);
const double NAVSTATE_PRINT_FG_ERRORS_THRESHOLD =
    mrpt::get_env<double>("NAVSTATE_PRINT_FG_ERRORS_THRESHOLD", 0.1);

using gtsam::symbol_shorthand::F;  // Frame of references (Pose3)
                                   // F(0): T_enu_to_map
                                   // F(i): T_map_to_odometry_frame_i
const auto symbol_T_enu_to_map         = F(0);
const auto symbol_T_map_to_odom_i_base = F(0);  // odom[i] = thisSymbol + i (with i>=1)

using gtsam::symbol_shorthand::T;  // Poses                          (Pose3)
using gtsam::symbol_shorthand::V;  // Lin velocity (body frame)      (Point3)
using gtsam::symbol_shorthand::W;  // Ang velocity (body frame)      (Point3)
//  TODO: IMU bias

constexpr unsigned int REFERENCE_FRAME_ID = 0;  // (for symbol_T_enu_to_map)

// -------- GtsamImpl -------

// everything related to gtsam is hidden in the public API via pimpl
// to reduce compilation dependencies, and build time and memory usage.
struct StateEstimationSmoother::GtsamImpl
{
    GtsamImpl() = default;

    // This is initialized in initialize(), once we have the parameters
    std::optional<gtsam::IncrementalFixedLagSmoother> smoother;

    // Queue of pending updates for incremental iSAM2:
    gtsam::NonlinearFactorGraph              newFactors;
    gtsam::Values                            newValues;
    gtsam::FixedLagSmoother::KeyTimestampMap newKeyStamps;
};

// -------- StateEstimationSmoother::State -------
StateEstimationSmoother::State::State()
    : gtsam(mrpt::make_impl<StateEstimationSmoother::GtsamImpl>())
{
}

// -------- StateEstimationSmoother -------
StateEstimationSmoother::StateEstimationSmoother()
{  //
    profiler_.setName("StateEstimationSmoother");
    ExecutableBase::setModuleInstanceName("StateEstimationSmoother");
}

void StateEstimationSmoother::initialize(const mrpt::containers::yaml& cfg)
{
    // Initialize parent:
    mola::NavStateFilter::initialize(cfg);

    this->mrpt::system::COutputLogger::setLoggerName("StateEstimationSmoother");

    MRPT_LOG_DEBUG_STREAM("initialize() called with:\n" << cfg << "\n");
    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");

    auto lck = mrpt::lockHelper(stateMutex_);

    // This also resets the GTSAM pimpl unique_ptr in state_
    reset();

    // Load params:
    params_.loadFrom(cfg["params"]);

    if (auto vizMods = ExecutableBase::findService<mola::VizInterface>(); !vizMods.empty())
    {
        visualizer_ = std::dynamic_pointer_cast<mola::VizInterface>(*vizMods.begin());
        if (visualizer_)
        {
            MRPT_LOG_DEBUG_STREAM("Connected to visualizer module");
        }
    }

    if (visualizer_)
    {
        this->mrpt::system::COutputLogger::logRegisterCallback(
            [&](std::string_view msg, const mrpt::system::VerbosityLevel level,
                std::string_view loggerName, const mrpt::Clock::time_point timestamp)
            {
                using namespace std::string_literals;

                if (!params_.visualization.show_console_messages)
                {
                    return;
                }

                if (level < this->getMinLoggingLevel())
                {
                    return;
                }

                visualizer_->output_console_message(
                    "["s + mrpt::system::timeLocalToString(timestamp) + "|"s +
                    mrpt::typemeta::enum2str(level) + " |"s + std::string(loggerName) + "]"s +
                    std::string(msg));
            });
    }

    // Forward parameters to GTSAM smoother & iSAM2:
    gtsam::ISAM2Params isam2Params;
    isam2Params.findUnusedFactorSlots = true;  // Important, must be set for fixed-lag smoother
    isam2Params.relinearizeThreshold  = 0.1;
    isam2Params.relinearizeSkip       = 1;
    // isam2Params.optimizationParams    = gtsam::ISAM2DoglegParams();

    state_.gtsam->smoother.emplace(params_.sliding_window_length, isam2Params);

    // Initialize georeference-related gtsam variables:
    // Even if not using a geo-referenced map, even if not using GPS sensors,
    // do define the T_enu_to_map transform variable, so it can be used for gravity-alignment
    // via IMU accelerometer, at least:
    auto           enu2map     = gtsam::Pose3::Identity();
    gtsam::Matrix6 enu2map_cov = gtsam::Matrix6::Identity() * mrpt::square(ENU2MAP_WEAK_SIGMA);

    if (params_.fixed_geo_reference.has_value())
    {
        state_.geo_reference = *params_.fixed_geo_reference;

        mrpt::gtsam_wrappers::to_gtsam_se3_cov6(
            state_.geo_reference->T_enu_to_map, enu2map, enu2map_cov);

        // Update into last_estimated_frames too, so estimated_T_enu_to_map() returns it:
        state_.last_estimated_frames[REFERENCE_FRAME_ID] = state_.geo_reference->T_enu_to_map;
    }

    // Initial value:
    state_.gtsam->newValues.insert(symbol_T_enu_to_map, enu2map);
    // Weak prior factor:
    state_.gtsam->newFactors.addPrior(symbol_T_enu_to_map, enu2map, enu2map_cov);
}

void StateEstimationSmoother::spinOnce()
{
    // At the predefined module rate, publish the current estimation, if we have any subscriber:
    if (!anyUpdateLocalizationSubscriber())
    {
        return;
    }

    auto lck = mrpt::lockHelper(stateMutex_);

    const auto tNowOpt = state_.get_current_extrapolated_stamp();
    if (!tNowOpt)
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "Cannot publish vehicle pose (no input data yet?)");
        return;
    }

    const auto nv = estimated_navstate(*tNowOpt, params_.reference_frame_name);
    if (!nv)
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "Cannot publish vehicle pose (stalled input data?)");
        return;
    }

    LocalizationUpdate lu;
    lu.child_frame     = params_.vehicle_frame_name;
    lu.reference_frame = params_.reference_frame_name;

    lu.method    = "state_estimator";
    lu.quality   = 1;
    lu.timestamp = *tNowOpt;
    lu.pose      = nv->pose.getPoseMean().asTPose();
    lu.cov       = nv->pose.cov_inv.inverse();

    MRPT_LOG_DEBUG_FMT(
        "[spinOnce] Publishing timely pose estimate: t=%f pose=%s", mrpt::Clock::toDouble(*tNowOpt),
        lu.pose.asString().c_str());

    advertiseUpdatedLocalization(lu);
}

void StateEstimationSmoother::reset()
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // reset:
    state_ = State();
}

void StateEstimationSmoother::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, const std::string& odomName)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // Integrates new wheels-based odometry observations into the estimator.
    //  This is a convenience method that internally ends up calling
    //  fuse_pose(), but computing the uncertainty of odometry increments
    //  according to a given motion model.

    mrpt::poses::CPose2D lastOdom;
    if (state_.last_wheels_odometry_name.has_value())
    {
        ASSERTMSG_(
            *state_.last_wheels_odometry_name == odomName,
            "More than one different 'odomName's received for wheels odometry!");

        ASSERT_(state_.last_wheels_odometry.has_value());
        lastOdom = *state_.last_wheels_odometry;
    }
    else
    {
        // This is the first time we have wheels odometry.
        lastOdom = odom.odometry;
    }
    // Use a probabilistic motion model:
    mrpt::obs::CActionRobotMovement2D odoAct;
    odoAct.motionModelConfiguration.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
    odoAct.motionModelConfiguration.gaussianModel.minStdPHI = 1e-3;
    odoAct.motionModelConfiguration.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.1);

    const auto odometryIncrement = odom.odometry - lastOdom;

    odoAct.computeFromOdometry(odometryIncrement, odoAct.motionModelConfiguration);

    mrpt::poses::CPose3DPDFGaussian newOdomPosePdf;
    newOdomPosePdf.copyFrom(*odoAct.poseChange);
    // Ensure as minimal uncertainty in all 3D DOFs to prevent numerical issues:
    newOdomPosePdf.cov.asEigen().diagonal().array() += 1e-4;

    // Convert probabilistic pose back to global "odom" frame for data fusion the in "odom" frame:
    newOdomPosePdf.changeCoordinatesReference(mrpt::poses::CPose3D(lastOdom));

    MRPT_LOG_DEBUG_FMT(
        "[fuse_odometry]: t=%f name=%s pose=%s poseChange=%s",
        mrpt::Clock::toDouble(odom.timestamp), odomName.c_str(), odom.odometry.asString().c_str(),
        odoAct.poseChange->getMeanVal().asString().c_str());

    // Save for next iteration:
    state_.last_wheels_odometry_name = odomName;
    state_.last_wheels_odometry      = odom.odometry;

    // Fuse this new probabilistic pose observation:
    this->fuse_pose(odom.timestamp, newOdomPosePdf, odomName);
}

void StateEstimationSmoother::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp(
        imu.timestamp, params_.imu_nearby_keyframe_stamp_tolerance);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_imu]: t=%f  this_kf_id=%zu ", mrpt::Clock::toDouble(imu.timestamp),
        static_cast<size_t>(this_kf_id));

    // Direct azimuth observation?
    // -------------------------------------------------
    if (imu.has(mrpt::obs::IMU_ORI_QUAT_W))
    {
        mrpt::math::CQuaternionDouble q;
        q.w(imu.get(mrpt::obs::IMU_ORI_QUAT_W));
        q.x(imu.get(mrpt::obs::IMU_ORI_QUAT_X));
        q.y(imu.get(mrpt::obs::IMU_ORI_QUAT_Y));
        q.z(imu.get(mrpt::obs::IMU_ORI_QUAT_Z));
        if (std::abs(q.norm() - 1.0) > 0.02)
        {
            MRPT_LOG_THROTTLE_WARN(5.0, "Ignoring non-normalized IMU orientation quaternion");
        }
        else
        {
            // correct heading:

            // Convert MRPT quaternion to GTSAM Rot3. (GTSAM uses w,x,y,z order)
            auto measuredRotation = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());

            // ENU is such yaw=0 ==> East. Correct this wrt Azimuth wrt true North:
            measuredRotation =
                gtsam::Rot3::Rz(mrpt::DEG2RAD(90.0 + params_.imu_attitude_azimuth_offset_deg)) *
                measuredRotation;

            const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(imu.sensorPose);

            // Create noise model for rotation (3 DOF: roll, pitch, yaw)
            auto rotationNoise = gtsam::noiseModel::Isotropic::Sigma(
                3, mrpt::DEG2RAD(params_.imu_attitude_sigma_deg));

            state_.gtsam->newFactors.emplace_shared<mola::factors::Pose3RotationFactor>(
                symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredRotation,
                rotationNoise);
        }
    }

    // Gravity-aligned acceleration observation?
    // -------------------------------------------------
    if (imu.has(mrpt::obs::IMU_X_ACC) && params_.imu_normalized_gravity_alignment_sigma > 0)
    {
        // TODO: Use ImuTransformer, etc.

        const gtsam::Vector3 measuredGravity = {
            imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC),
            imu.get(mrpt::obs::IMU_Z_ACC)};

        // Some IMU drivers publishes normalized acc:
        if (std::abs(measuredGravity.norm() - 9.8) < 2.0 ||
            std::abs(measuredGravity.norm() - 1.0) < 0.2)
        {
            const gtsam::Vector3 measuredGravityNormalized = measuredGravity.normalized();

            const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(imu.sensorPose);

            // Create noise model for gravity alignment:
            auto accNoise = gtsam::noiseModel::Isotropic::Sigma(
                3, mrpt::DEG2RAD(params_.imu_normalized_gravity_alignment_sigma));

            state_.gtsam->newFactors.emplace_shared<mola::factors::MeasuredGravityFactor>(
                symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredGravityNormalized,
                accNoise);
        }
    }
}

void StateEstimationSmoother::fuse_gnss(const mrpt::obs::CObservationGPS& gps)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    if (!gps.has_GGA_datum())
    {
        MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading since it has no GGA data.");
        return;
    }
    const auto& gga = gps.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

    if (!gga.fields.fix_quality)
    {
        MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading. GGA has no valid datum (fix_quality)");
        return;
    }

    const auto geoCoords = gga.getAsStruct<mrpt::topography::TGeodeticCoords>();

    std::optional<mrpt::topography::TGeodeticCoords> refGeoCoords;

    // Determine the reference to use
    if (state_.geo_reference.has_value())
    {
        // We have a finalized reference (either fixed or already estimated)
        refGeoCoords = state_.geo_reference->geo_coord;
    }
    else if (params_.estimate_geo_reference)
    {
        // We are still in the "tentative" phase
        if (!state_.tentative_geo_coord_reference)
        {
            state_.tentative_geo_coord_reference = geoCoords;

            MRPT_LOG_DEBUG_STREAM(
                "[fuse_gnss]: Defining as geodetic reference: lat="
                << geoCoords.lat.getAsString() << ", lon=" << geoCoords.lon.getAsString()
                << ", h=" << geoCoords.height);
        }
        refGeoCoords = state_.tentative_geo_coord_reference;
    }

    if (!refGeoCoords.has_value())
    {
        MRPT_LOG_DEBUG(
            "[fuse_gnss]: Ignoring reading; no fixed or tentative geo-reference available.");
        return;
    }

    if (!gps.covariance_enu.has_value())
    {
        MRPT_LOG_THROTTLE_WARN(
            5.0, "Discarding GNSS (GPS) reading since it does not have ENU covariance.");
        return;
    }

    mrpt::math::TPoint3D ENU_point;
    mrpt::topography::geodeticToENU_WGS84(geoCoords, ENU_point, *refGeoCoords);

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp(
        gps.timestamp, params_.gnss_nearby_keyframe_stamp_tolerance);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_gnss]: t=%f this_kf_id=%zu ENU=%s", mrpt::Clock::toDouble(gps.timestamp),
        static_cast<size_t>(this_kf_id), ENU_point.asString().c_str());

    // Add geo-ref factor:
    const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPoint3(gps.sensorPose.translation());
    const auto observedEnu     = mrpt::gtsam_wrappers::toPoint3(ENU_point);
    const auto enuNoise = gtsam::noiseModel::Gaussian::Covariance(gps.covariance_enu->asEigen());
    auto       enuNoiseRobust = gtsam::noiseModel::Robust::Create(
              gtsam::noiseModel::mEstimator::Huber::Create(1.5), enuNoise);

    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorGnssMapEnu>(
        symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, observedEnu, enuNoiseRobust);
}

void StateEstimationSmoother::fuse_pose(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // get this numerical frame_id :
    const auto frame_id_idx = add_or_get_odom_frame_id(frame_id);

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp(timestamp);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_pose]: kf_idx=%zu t=%f frame='%s' (idx=%zu) p=%s sigmas=%.02e %.02e %.02e (m) %.02e "
        "%.02e %.02e (deg)",
        static_cast<std::size_t>(this_kf_id), mrpt::Clock::toDouble(timestamp), frame_id.c_str(),
        static_cast<std::size_t>(frame_id_idx), pose.mean.asString().c_str(),
        std::sqrt(pose.cov(0, 0)), std::sqrt(pose.cov(1, 1)), std::sqrt(pose.cov(2, 2)),
        mrpt::RAD2DEG(std::sqrt(pose.cov(3, 3))), mrpt::RAD2DEG(std::sqrt(pose.cov(4, 4))),
        mrpt::RAD2DEG(std::sqrt(pose.cov(5, 5))));

    // numerical sanity:
    for (int i = 0; i < 6; i++)
    {
        ASSERT_GT_(pose.cov(i, i), .0);
    }

    // Add factor:
    gtsam::Pose3   pose_out;
    gtsam::Matrix6 cov_out;
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(pose, pose_out, cov_out);

    // TODO: robust factors here?

    // reference frame ("map") or "odom_i"?
    if (frame_id_idx == REFERENCE_FRAME_ID)
    {
        // ref is "map":
        state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(this_kf_id), pose_out, gtsam::noiseModel::Gaussian::Covariance(cov_out));
    }
    else
    {
        // ref is an odometry frame:
        state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            symbol_T_map_to_odom_i_base + frame_id_idx, T(this_kf_id), pose_out,
            gtsam::noiseModel::Gaussian::Covariance(cov_out));
    }
}

void StateEstimationSmoother::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    const gtsam::Vector3 v    = {twist.vx, twist.vy, twist.vz};
    const gtsam::Vector3 w    = {twist.wx, twist.wy, twist.wz};
    gtsam::Matrix3       vCov = twistCov.asEigen().block<3, 3>(0, 0);
    gtsam::Matrix3       wCov = twistCov.asEigen().block<3, 3>(3, 3);

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp(timestamp);

    {
        auto                                noiseV = gtsam::noiseModel::Gaussian::Covariance(vCov);
        gtsam::noiseModel::Base::shared_ptr robNoiseV;
#if 0
        if (params_.robust_param > 0)
        {
            robNoiseV = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::GemanMcClure::Create(params_.robust_param), noiseV);
        }
        else
#endif
        {
            robNoiseV = noiseV;
        }

        state_.gtsam->newFactors.addPrior(V(this_kf_id), v, robNoiseV);
    }
    {
        auto                                noiseW = gtsam::noiseModel::Gaussian::Covariance(wCov);
        gtsam::noiseModel::Base::shared_ptr robNoiseW;
#if 0
        if (params_.robust_param > 0)
        {
            robNoiseW = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::GemanMcClure::Create(params_.robust_param), noiseW);
        }
        else
#endif
        {
            robNoiseW = noiseW;
        }

        state_.gtsam->newFactors.addPrior(W(this_kf_id), w, robNoiseW);
    }

    MRPT_LOG_DEBUG_FMT(
        "[fuse_twist]: t=%f this_kf_id=%zu twist=%s sigmas=%.02e %.02e %.02e (m) %.02e %.02e "
        "%.02e (deg)",
        mrpt::Clock::toDouble(timestamp), static_cast<std::size_t>(this_kf_id),
        twist.asString().c_str(), std::sqrt(twistCov(0, 0)), std::sqrt(twistCov(1, 1)),
        std::sqrt(twistCov(2, 2)), mrpt::RAD2DEG(std::sqrt(twistCov(3, 3))),
        mrpt::RAD2DEG(std::sqrt(twistCov(4, 4))), mrpt::RAD2DEG(std::sqrt(twistCov(5, 5))));
}

std::optional<NavState> StateEstimationSmoother::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, const std::string& frame_id)
{
    // 1) Make sure we processed all pending sensor data, and have updated the cached values from
    //    GTSAM values
    process_pending_gtsam_updates();

    // 2) Get the vehicle state from cached optimized values:
    // Look for the closest frame and extrapolate.
    auto lck = mrpt::lockHelper(stateMutex_);

    std::optional<double>        closestFrameDt;
    double                       closestFrameDtSigned = 0;
    std::optional<frame_index_t> closesFrameIdx;

    const auto closestPrior = find_before_after(timestamp, true);
    for (const auto& it : {closestPrior.first, closestPrior.second})
    {
        if (it == state_.stamp2frame_index.getDirectMap().end())
        {
            continue;
        }
        const auto& [existing_t, frame_idx] = *it;

        const double dt    = mrpt::system::timeDifference(existing_t, timestamp);
        const double dtAbs = std::abs(dt);
        if (!closestFrameDt.has_value() || dtAbs < *closestFrameDt)
        {
            closestFrameDt       = dtAbs;
            closesFrameIdx       = frame_idx;
            closestFrameDtSigned = dt;
        }
    }

    // Check maximum extrapolation time:
    if (!closesFrameIdx.has_value() || closestFrameDt > params_.max_time_to_use_velocity_model)
    {
        MRPT_LOG_DEBUG_FMT(
            "[estimated_navstate] Could not find any nearby frame near requested t=%.03f",
            mrpt::Clock::toDouble(timestamp));
        return {};
    }

    // Recover the closest state *in the reference frame*:
    const NavState retKf = get_latest_state_and_covariance(*closesFrameIdx);

    MRPT_TODO("Implement probabilistic extrapolation");
    // For now, approximate extrapolation only:
    NavState ret = retKf;

    mrpt::math::CVectorFixed<double, 6> twistDt;
    twistDt[0] = ret.twist.vx;
    twistDt[1] = ret.twist.vy;
    twistDt[2] = ret.twist.vz;
    twistDt[3] = ret.twist.wx;
    twistDt[4] = ret.twist.wy;
    twistDt[5] = ret.twist.wz;
    twistDt *= closestFrameDtSigned;
    // SE(3) pose composition for extrapolating. TODO: Missing update cov!
    ret.pose.mean = ret.pose.mean + mrpt::poses::Lie::SE<3>::exp(twistDt);

    // Approximate uncertainty growth due to random walk:
    {
        auto twist_cov = ret.twist_inv_cov.inverse_LLt();
        for (int i = 0; i < 3; i++)
        {
            twist_cov(0 + i, 0 + i) +=
                mrpt::square(params_.sigma_random_walk_acceleration_linear * closestFrameDtSigned);

            twist_cov(3 + i, 3 + i) +=
                mrpt::square(params_.sigma_random_walk_acceleration_angular * closestFrameDtSigned);
        }
        ret.twist_inv_cov = twist_cov.inverse_LLt();
    }

    // 3) Convert pose to the requested frame_id:
    if (frame_id != params_.reference_frame_name)
    {
        // Transform:
        const auto requestedFrameIdx    = state_.known_odom_frames.direct(frame_id);
        const auto posePdfFrame_wrt_map = state_.last_estimated_frames.at(requestedFrameIdx);

        mrpt::poses::CPose3DPDFGaussianInf posePdfFrame_wrt_map_inf;
        posePdfFrame_wrt_map_inf.copyFrom(posePdfFrame_wrt_map);

        // Probabilistic inverse pose composition:
        // TO-DO: Is it worth including the cross-covariances...?
        ret.pose = ret.pose - posePdfFrame_wrt_map_inf;
    }

    return ret;
}

std::set<std::string> StateEstimationSmoother::known_odometry_frame_ids()
{
    auto lck = mrpt::lockHelper(stateMutex_);

    std::set<std::string> ret;
    for (const auto& [name, id] : state_.known_odom_frames.getDirectMap())
    {
        ret.insert(name);
    }

    return ret;
}

void StateEstimationSmoother::onNewObservation(const CObservation::ConstPtr& o)
{
    const ProfilerEntry tle(profiler_, "onNewObservation");

    ASSERT_(o);

    // IMU:
    if (auto obsIMU = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o); obsIMU)
    {
        if (std::regex_match(
                o->sensorLabel,
                state_.do_process_imu_labels_re.get_regex(params_.do_process_imu_labels_re)))
        {
            this->fuse_imu(*obsIMU);
        }
        else
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping IMU reading labeled '%s' for not passing regex", o->sensorLabel.c_str());
        }
    }
    // Odometry source:
    else if (auto obsOdom = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o);
             obsOdom)
    {
        if (std::regex_match(
                o->sensorLabel, state_.do_process_odometry_labels_re.get_regex(
                                    params_.do_process_odometry_labels_re)))
        {
            this->fuse_odometry(*obsOdom, o->sensorLabel);
        }
        else
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping odometry reading labeled '%s' for not passing regex",
                o->sensorLabel.c_str());
        }
    }
    // Robot pose wrt "map":
    else if (auto obsPose = std::dynamic_pointer_cast<const mrpt::obs::CObservationRobotPose>(o);
             obsPose)
    {
        auto sensedSensorPose = obsPose->pose;
        if (obsPose->sensorPose != mrpt::poses::CPose3D())
        {
            sensedSensorPose =
                sensedSensorPose + mrpt::poses::CPose3DPDFGaussian(-obsPose->sensorPose);
        }

        this->fuse_pose(obsPose->timestamp, sensedSensorPose, params_.reference_frame_name);
    }
    // GNSS source:
    else if (auto obsGPS = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o); obsGPS)
    {
        if (std::regex_match(
                o->sensorLabel,
                state_.do_process_gnss_labels_re.get_regex(params_.do_process_gnss_labels_re)))
        {
            this->fuse_gnss(*obsGPS);
        }
        else
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping GNSS reading labeled '%s' for not passing regex", o->sensorLabel.c_str());
        }
    }
    else
    {
        MRPT_LOG_THROTTLE_DEBUG_FMT(
            10.0,
            "Do not know how to handle incoming observation label='%s' "
            "class='%s'",
            o->sensorLabel.c_str(), o->GetRuntimeClass()->className);
    }
}

/// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
void StateEstimationSmoother::addFactor(const AbsFactorConstVelKinematics& f)
{
    MRPT_LOG_DEBUG_STREAM(
        "[addFactor] FactorConstVelKinematics: " << f.from_kf << " ==> " << f.to_kf
                                                 << " dt=" << f.deltaTime);

    // Add const-vel factor to gtsam itself:
    double dt = f.deltaTime;

    // trick to easily handle queries on exactly an existing keyframe:
    if (dt == 0)
    {
        dt = 1e-5;
    }

    ASSERT_GT_(dt, 0.);

    // errors in constant vel:
    const double std_lin_vel = params_.sigma_random_walk_acceleration_linear;
    const double std_ang_vel = params_.sigma_random_walk_acceleration_angular;

    if (dt > params_.time_between_frames_to_warning)
    {
        MRPT_LOG_WARN_FMT("Constant-velocity kinematics factor added for large dT=%.03f s.", dt);
    }

    // 1) Add GTSAM factors for constant velocity model
    // -------------------------------------------------
    const auto kTi  = T(f.from_kf);
    const auto kTj  = T(f.to_kf);
    const auto kbVi = V(f.from_kf);
    const auto kbVj = V(f.to_kf);
    const auto kbWi = W(f.from_kf);
    const auto kbWj = W(f.to_kf);

    // See line 3 of eq (4) in the MOLA RSS2019 paper
    // Modify to use velocity in local frame: reuse FactorConstLocalVelocity
    // here too:
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, std_ang_vel * dt));

    // 2) Add kinematics / numerical integration factor
    // ---------------------------------------------------
    auto noise_kinematicsPosition =
        gtsam::noiseModel::Isotropic::Sigma(3, params_.sigma_integrator_position);

    auto noise_kinematicsOrientation =
        gtsam::noiseModel::Isotropic::Sigma(3, params_.sigma_integrator_orientation);

    // Impl. line 2 of eq (1) in the MOLA RSS2019 paper
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorTrapezoidalIntegratorPose>(
        kTi, kbVi, kTj, kbVj, dt, noise_kinematicsPosition);

    // Impl. line 1 of eq (4) in the MOLA RSS2019 paper.
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorAngularVelocityIntegrationPose>(
        kTi, kbWi, kTj, dt, noise_kinematicsOrientation);
}

void StateEstimationSmoother::addFactor(const AbsFactorTricycleKinematics& f)
{
    MRPT_LOG_DEBUG_STREAM(
        "[addFactor] FactorTricycleKinematics: " << f.from_kf << " ==> " << f.to_kf
                                                 << " dt=" << f.deltaTime);

    // Add const-vel factor to gtsam itself:
    double dt = f.deltaTime;

    // trick to easily handle queries on exactly an existing keyframe:
    if (dt == 0)
    {
        dt = 1e-5;
    }

    ASSERT_GT_(dt, 0.);

    // errors in constant vel:
    const double std_lin_vel = params_.sigma_random_walk_acceleration_linear;
    const double std_ang_vel = params_.sigma_random_walk_acceleration_angular;

    if (dt > params_.time_between_frames_to_warning)
    {
        MRPT_LOG_WARN_FMT("Tricycle kinematics factor added for large dT=%.03f s.", dt);
    }

    // 1) Add GTSAM factors for constant velocity model
    // -------------------------------------------------
    const auto kTi  = T(f.from_kf);
    const auto kTj  = T(f.to_kf);
    const auto kbVi = V(f.from_kf);
    const auto kbVj = V(f.to_kf);
    const auto kbWi = W(f.from_kf);
    const auto kbWj = W(f.to_kf);

    // See line 3 of eq (4) in the MOLA RSS2019 paper
    // Modify to use velocity in local frame: reuse FactorConstLocalVelocity
    // here too:
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, std_ang_vel * dt));

    // In the tricycle model, body v_y must be zero:
    {
        const Eigen::Vector3d sigmas = {
            TRICYCLE_LARGE_SIGMAS, std_lin_vel * dt, TRICYCLE_LARGE_SIGMAS};

        state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
            kbVj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }
    // In the tricycle model, body w_x,w_y must be zero:
    {
        const Eigen::Vector3d sigmas = {std_ang_vel * dt, std_ang_vel * dt, TRICYCLE_LARGE_SIGMAS};

        state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
            kbWj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }

    // 2) Add kinematics / numerical integration factor
    // ---------------------------------------------------
    gtsam::Vector6 sigmas;
    const auto     sigmaPos   = params_.sigma_integrator_position;
    const auto     sigmaAngle = params_.sigma_integrator_orientation;
    sigmas << sigmaAngle, sigmaAngle, sigmaAngle, sigmaPos, sigmaPos, sigmaPos;

    auto noise_kinematics = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    // (To be written in a report/paper!)
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorTricycleKinematic>(
        kTi, kbVi, kbWi, kTj, dt, noise_kinematics);
}

void StateEstimationSmoother::delete_too_old_entries()
{
    // auto lck = mrpt::lockHelper(stateMutex_); // this is assumed to be acquired by caller

    // Remove really old entries in our bimap. GTSAM fixed lag handles removing actual factors.
    const double newestTime =
        mrpt::Clock::toDouble(state_.stamp2frame_index.getDirectMap().rbegin()->first);
    const double minTime = newestTime - params_.sliding_window_length;

    std::set<mrpt::Clock::time_point> stamps_to_erase;
    std::set<frame_index_t>           ids_to_erase;
    for (const auto& [existing_t, frame_idx] : state_.stamp2frame_index)
    {
        const double t_existing = mrpt::Clock::toDouble(existing_t);
        if (t_existing < minTime)
        {
            stamps_to_erase.insert(existing_t);
            ids_to_erase.insert(frame_idx);
        }
    }
    for (const auto& t_erase : stamps_to_erase)
    {
        state_.stamp2frame_index.erase_by_key(t_erase);
    }
    for (const auto& idx : ids_to_erase)
    {
        state_.last_estimated_states.erase(idx);
    }
}

// Creates a new frame index for timestamp t, or returns the existing one if close enough.
// This also is in charge of the complex task of finding nearby existing frames and adding the
// kinematic factors to ensure smooth motion estimation.
StateEstimationSmoother::frame_index_t StateEstimationSmoother::create_or_get_keyframe_by_timestamp(
    const mrpt::Clock::time_point& t, const std::optional<double>& overrideCloseEnough)
{
    const auto tle = mola::ProfilerEntry(profiler_, "create_or_get_keyframe_by_timestamp");

    auto lck = mrpt::lockHelper(stateMutex_);

    const double threshold = overrideCloseEnough ? *overrideCloseEnough
                                                 : params_.min_time_difference_to_create_new_frame;

    // See if we have an existing frame index close enough to t:
    const auto closestPrior = find_before_after(t, true);
    for (const auto& it : {closestPrior.first, closestPrior.second})
    {
        if (it == state_.stamp2frame_index.getDirectMap().end())
        {
            continue;
        }
        const auto& [existing_t, frame_idx] = *it;

        const double dt = std::abs(mrpt::system::timeDifference(existing_t, t));

        if (dt < threshold)
        {
            return frame_idx;
        }
    }

    // Create a new one:
    const auto newFrameIdx = state_.next_frame_index++;
    state_.stamp2frame_index.insert(t, newFrameIdx);

    // As we create a new timely keyframe, update what's the last time we created such new frame:
    state_.last_observation_stamp           = t;
    state_.last_observation_wallclock_stamp = mrpt::Clock::now();

    // Look for the closest existing frames, and create kinematic pairs if they don't exist yet:
    const auto closestPost = find_before_after(t, false);

    // Create new GTSAM symbols for this keyframe:
    initialize_new_frame(newFrameIdx, closestPost);

    if (closestPost.first != state_.stamp2frame_index.getDirectMap().end())
    {
        const auto [t_before, idx_before] = *closestPost.first;
        MRPT_LOG_DEBUG_FMT(
            "[add_or_get_timestamp_frame_index] New frame created: idx=%zu, t_before=%f (idx=%zu)",
            static_cast<size_t>(newFrameIdx), mrpt::Clock::toDouble(t_before),
            static_cast<size_t>(idx_before));

        // Add kinematic factors:
        add_kinematic_factor_between(idx_before, newFrameIdx);
    }

    if (closestPost.second != state_.stamp2frame_index.getDirectMap().end())
    {
        const auto [t_after, idx_after] = *closestPost.second;
        MRPT_LOG_DEBUG_FMT(
            "[add_or_get_timestamp_frame_index] New frame created: idx=%zu, t_after=%f (idx=%zu)",
            static_cast<size_t>(newFrameIdx), mrpt::Clock::toDouble(t_after),
            static_cast<size_t>(idx_after));

        // Add kinematic factors:
        add_kinematic_factor_between(newFrameIdx, idx_after);
    }

    // Remove really old entries in our bimap. GTSAM fixed lag handles removing actual factors.
    delete_too_old_entries();

    return newFrameIdx;
}

// Creates or returns the existing ID, for an odometry frame_id:
StateEstimationSmoother::odometry_frameid_t StateEstimationSmoother::add_or_get_odom_frame_id(
    const std::string& frame_id_name)
{
    const auto tle = mola::ProfilerEntry(profiler_, "add_or_get_odom_frame_id");

    // F(0): is special, it's the reference frame ("map"), not a floating "odometry" frame
    if (frame_id_name == params_.reference_frame_name)
    {
        return REFERENCE_FRAME_ID;
    }

    ASSERT_NOT_EQUAL_(frame_id_name, params_.vehicle_frame_name);
    ASSERT_NOT_EQUAL_(frame_id_name, params_.enu_frame_name);

    // auto lck = mrpt::lockHelper(stateMutex_); // acquired by caller

    // Existing frame?
    if (auto it = state_.known_odom_frames.find_key(frame_id_name);
        it != state_.known_odom_frames.getDirectMap().end())
    {
        return it->second;
    }

    // New one: starting at "1" (0=reserved for "map")
    const auto newId = static_cast<odometry_frameid_t>(state_.known_odom_frames.size()) + 1;
    state_.known_odom_frames.insert(frame_id_name, newId);

    // Initialize gtsam symbol and prior factor for the new frame:
    const gtsam::Pose3 initFramePose = gtsam::Pose3::Identity();

    ASSERT_GE_(newId, 1);

    state_.gtsam->newValues.insert(symbol_T_map_to_odom_i_base + newId, initFramePose);
    state_.gtsam->newFactors.addPrior(
        symbol_T_map_to_odom_i_base + newId, initFramePose,
        gtsam::noiseModel::Isotropic::Sigma(6, INIT_ODOM_FRAME_POSE_SIGMA));

    return newId;
}

void StateEstimationSmoother::process_pending_gtsam_updates()
{
    const auto tle = mola::ProfilerEntry(profiler_, "process_pending_gtsam_updates");

    auto lck = mrpt::lockHelper(stateMutex_);

    // Even if we have no new factors/values, do update the stamps of "persistent" variables:
    if (state_.last_observation_stamp.has_value())
    {
        const auto lastObservationStamp_sec = mrpt::Clock::toDouble(*state_.last_observation_stamp);

        state_.gtsam->newKeyStamps[symbol_T_enu_to_map] = lastObservationStamp_sec;
        for (const auto& [_, frameId] : state_.known_odom_frames)
        {
            state_.gtsam->newKeyStamps[symbol_T_map_to_odom_i_base + frameId] =
                lastObservationStamp_sec;
        }
    }

    if (NAVSTATE_PRINT_FG)
    {
        state_.gtsam->smoother->getFactors().print("EXISTING FACTORS:");
        state_.gtsam->newFactors.print("NEW FACTORS:");
        state_.gtsam->newValues.print("NEW VALUES:");
#if 0
        fg.saveGraph("fg.dot");
#endif
    }

    auto& smoother = *state_.gtsam->smoother;

    // Update the smoother with pending factors/values:
    if (!state_.gtsam->newFactors.empty() || !state_.gtsam->newValues.empty() ||
        !state_.gtsam->newKeyStamps.empty())
    {
        smoother.update(
            state_.gtsam->newFactors, state_.gtsam->newValues, state_.gtsam->newKeyStamps);
    }

    // Optional: Perform extra internal iterations for better accuracy
    for (unsigned int i = 1; i < params_.additional_isam2_update_steps; ++i)
    {
        smoother.update();
    }

    // Print debug info:
    MRPT_LOG_DEBUG_STREAM(
        "[process_pending_gtsam_updates] After update: "
        << smoother.getFactors().size() << " factors, " << smoother.getFactors().nrFactors()
        << " nr factors. New factors=" << state_.gtsam->newFactors.size()
        << ", new values=" << state_.gtsam->newValues.size());

    const auto optValues = smoother.calculateEstimate();

    // Retrieve the latest estimate and save it into "state_.last_estimated_state":
    for (auto& [kfIdx, kf] : state_.last_estimated_states)
    {
        const auto pose = optValues.at<gtsam::Pose3>(T(kfIdx));
        const auto linV = optValues.at<gtsam::Vector3>(V(kfIdx));
        const auto angV = optValues.at<gtsam::Vector3>(W(kfIdx));

        kf.pose  = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(pose));
        kf.twist = {linV.x(), linV.y(), linV.z(), angV.x(), angV.y(), angV.z()};

        if (params_.enforce_planar_motion)
        {
            enforce_planar_pose(kf.pose);
            enforce_planar_twist(kf.twist);
        }
    }

    // Retrieve latest enu_to_map for geo-referencing:
    if (params_.estimate_geo_reference)
    {
        const auto T_enu_to_map     = optValues.at<gtsam::Pose3>(symbol_T_enu_to_map);
        const auto T_enu_to_map_cov = smoother.marginalCovariance(symbol_T_enu_to_map);

        auto& pdf = state_.last_estimated_frames[REFERENCE_FRAME_ID];

        pdf.mean = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T_enu_to_map));
        pdf.cov  = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T_enu_to_map_cov);

        // Convert info matrix to covariance
        if (!state_.stamp2frame_index.empty())
        {
            const auto& latestIt       = state_.stamp2frame_index.getDirectMap().rbegin();
            const auto  latestFrameIdx = latestIt->second;

            const auto poseCov =
                gtsam::Matrix6(state_.gtsam->smoother->marginalCovariance(T(latestFrameIdx)));
            mrpt::math::CMatrixDouble66 cov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(poseCov);

            // Check MAP->BASE_LINK position uncertainty (diagonal elements 0,1,2 are x,y,z)
            const double pos_sigma_x   = std::sqrt(cov(0, 0));
            const double pos_sigma_y   = std::sqrt(cov(1, 1));
            const double pos_sigma_z   = std::sqrt(cov(2, 2));
            const double max_pos_sigma = std::max({pos_sigma_x, pos_sigma_y, pos_sigma_z});

            // Check MAP->BASE_LINK orientation uncertainty (diagonal elements 3,4,5 are
            // yaw,pitch,roll)
            const double ori_sigma_yaw   = std::sqrt(cov(3, 3));
            const double ori_sigma_pitch = std::sqrt(cov(4, 4));
            const double ori_sigma_roll  = std::sqrt(cov(5, 5));
            const double max_ori_sigma_rad =
                std::max({ori_sigma_yaw, ori_sigma_pitch, ori_sigma_roll});
            const double max_ori_sigma_deg = mrpt::RAD2DEG(max_ori_sigma_rad);

            // Check ENU->MAP position uncertainty (diagonal elements 0,1,2 are x,y,z)
            const double em_pos_sigma_x = std::sqrt(pdf.cov(0, 0));
            const double em_pos_sigma_y = std::sqrt(pdf.cov(1, 1));
            const double em_pos_sigma_z = std::sqrt(pdf.cov(2, 2));
            const double em_max_pos_sigma =
                std::max({em_pos_sigma_x, em_pos_sigma_y, em_pos_sigma_z});

            // Check ENU->MAP orientation uncertainty (diagonal elements 3,4,5 are yaw,pitch,roll)
            const double em_ori_sigma_yaw   = std::sqrt(pdf.cov(3, 3));
            const double em_ori_sigma_pitch = std::sqrt(pdf.cov(4, 4));
            const double em_ori_sigma_roll  = std::sqrt(pdf.cov(5, 5));
            const double em_max_ori_sigma_rad =
                std::max({em_ori_sigma_yaw, em_ori_sigma_pitch, em_ori_sigma_roll});
            const double em_max_ori_sigma_deg = mrpt::RAD2DEG(em_max_ori_sigma_rad);

            // Check against thresholds
            const bool converged = (std::max(max_pos_sigma, em_max_pos_sigma) <=
                                    params_.convergence_max_position_sigma) &&
                                   (std::max(max_ori_sigma_deg, em_max_ori_sigma_deg) <=
                                    params_.convergence_max_orientation_sigma_deg);

            MRPT_LOG_DEBUG_FMT(
                "[process_pending_gtsam_updates] Has converged georeferencing? %s: "
                "pos_sigmas=(%.3f,%.3f,%.3f) m, "
                "enu_pos_sigmas=(%.3f,%.3f,%.3f) m, "
                "ori_sigmas=(%.2f,%.2f,%.2f) deg, "
                "enu_ori_sigmas=(%.2f,%.2f,%.2f) deg, "
                "thresholds=(%.3f m, %.2f deg)",
                converged ? "YES" : "NO", pos_sigma_x, pos_sigma_y, pos_sigma_z, em_pos_sigma_x,
                em_pos_sigma_y, em_pos_sigma_z, mrpt::RAD2DEG(ori_sigma_yaw),
                mrpt::RAD2DEG(ori_sigma_pitch), mrpt::RAD2DEG(ori_sigma_roll),
                mrpt::RAD2DEG(em_ori_sigma_yaw), mrpt::RAD2DEG(em_ori_sigma_pitch),
                mrpt::RAD2DEG(em_ori_sigma_roll), params_.convergence_max_position_sigma,
                params_.convergence_max_orientation_sigma_deg);

            if (converged && state_.tentative_geo_coord_reference.has_value())
            {
                state_.geo_reference.emplace();
                state_.geo_reference->geo_coord    = state_.tentative_geo_coord_reference.value();
                state_.geo_reference->T_enu_to_map = pdf;
                state_.estimated_georef_published  = false;  // so it's re-published
            }
        }
    }

    // retrieve odometry frames:
    for (const auto& [_, odomFrameIdx] : state_.known_odom_frames)
    {
        const auto symbolOdom        = symbol_T_map_to_odom_i_base + odomFrameIdx;
        const auto T_map2_odom_i     = optValues.at<gtsam::Pose3>(symbolOdom);
        const auto T_map2_odom_i_cov = smoother.marginalCovariance(symbolOdom);

        auto& pdf = state_.last_estimated_frames[odomFrameIdx];

        pdf.mean = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T_map2_odom_i));
        pdf.cov  = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T_map2_odom_i_cov);
    }

    if (NAVSTATE_PRINT_FG)
    {
        smoother.getFactors().print("FG:\n");
        optValues.print("Optimized values:\n");
    }

    if (NAVSTATE_PRINT_FG_ERRORS)
    {
        smoother.getFactors().printErrors(
            optValues, "Errors for optimized values:", gtsam::DefaultKeyFormatter,
            [](const gtsam::Factor* /*factor*/, double whitenedError, size_t /*index*/)
            { return whitenedError > NAVSTATE_PRINT_FG_ERRORS_THRESHOLD; });
    }

    if (isLoggingLevelVisible(mrpt::system::LVL_DEBUG) && !smoother.getFactors().empty())
    {
        const double final_rmse = std::sqrt(
            smoother.getFactors().error(optValues) /
            static_cast<double>(smoother.getFactors().size()));

        MRPT_LOG_DEBUG_STREAM("[process_pending_gtsam_updates] iSAM2 final RMSE: " << final_rmse);
    }

    // Clear pending updates:
    state_.gtsam->newFactors.resize(0);
    state_.gtsam->newValues.clear();
    state_.gtsam->newKeyStamps.clear();

    // Check convergence for initialization from GNSS / georeferencing.
    // If we just converged and should publish geo-ref, do it now:
    if (params_.estimate_geo_reference && params_.publish_estimated_georef_on_convergence &&
        !state_.estimated_georef_published)
    {
        // Publish the estimated georeferencing
        publishEstimatedGeoreferencing();
    }
}

StateEstimationSmoother::pair_nearby_frame_iterators_t StateEstimationSmoother::find_before_after(
    const mrpt::Clock::time_point& t, bool allow_exact_match)
{
    const auto& stamp2frame = state_.stamp2frame_index.getDirectMap();

    using Iterator = std::map<mrpt::Clock::time_point, frame_index_t>::const_iterator;

    if (stamp2frame.empty())
    {
        return {stamp2frame.end(), stamp2frame.end()};
    }

    // upper_bound finds the first element whose key is > t. This is the 'after' element.
    Iterator after = stamp2frame.upper_bound(t);

    if (!allow_exact_match)
    {
        // Now determine the 'before' element.
        Iterator before;
        if (after == stamp2frame.begin())
        {
            // Case A: t is smaller than ALL keys.
            // No element before t. 'after' is the first element.
            before = stamp2frame.end();
        }
        else
        {
            // Case B: t is greater than or equal to some key(s).
            // 'before' is the element immediately preceding 'after'.
            before = std::prev(after);
        }

        // Now refine the 'after' iterator based on an exact match with the 'before' iterator.
        // If the 'before' element's key is exactly 't', then 'before' is the exact match.
        // The element *after* it is what upper_bound already found.
        // If the 'before' element's key is < 't', then 'before' is the correct predecessor.
        // Check if t is an exact match:
        if (before != stamp2frame.end() && before->first == t)
        {
            // An exact match for t exists.
            // 'before' is the exact match element.
            // The element BEFORE the exact match is its predecessor, if it exists.
            Iterator element_before_match =
                (before == stamp2frame.begin()) ? stamp2frame.end() : std::prev(before);

            // The element AFTER the exact match is what upper_bound already found (the current
            // 'after').
            return {element_before_match, after};
        }

        // General case: t lies strictly between 'before' and 'after' (or is smaller/larger than
        // all). The iterators 'before' and 'after' are correct as computed above.
        return {before, after};
    }

    // case: allow_exact_match is "true"

    // If 'after' is the beginning, t is smaller than all keys.
    if (after == stamp2frame.begin())
    {
        return {stamp2frame.end(), after};  // Case A: No 'before' element
    }

    // Otherwise, 'before' is the element immediately preceding 'after'.
    Iterator before = std::prev(after);

    // This pair correctly handles:
    // 1. t between K_i and K_j: before = K_i, after = K_j
    // 2. t exactly matches K_i: before = K_i, after = K_{i+1}
    // 3. t larger than all: before = K_max, after = end()

    return {before, after};
}

// Pick the closest of the two possible frames, or none if both iterators are end()
std::optional<StateEstimationSmoother::frame_index_t> StateEstimationSmoother::pick_closest(
    const StateEstimationSmoother::pair_nearby_frame_iterators_t& closestFrames,
    const mrpt::Clock::time_point&                                stamp) const
{
    const auto& [before, after] = closestFrames;

    // Both iterators are end(), no frames available
    if (before == state_.stamp2frame_index.getDirectMap().end() &&
        after == state_.stamp2frame_index.getDirectMap().end())
    {
        return std::nullopt;
    }

    // Only 'after' is available
    if (before == state_.stamp2frame_index.getDirectMap().end())
    {
        return after->second;
    }

    // Only 'before' is available
    if (after == state_.stamp2frame_index.getDirectMap().end())
    {
        return before->second;
    }

    // Both available, pick the closest by timestamp
    const double dtBefore = std::abs(mrpt::system::timeDifference(stamp, before->first));
    const double dtAfter  = std::abs(mrpt::system::timeDifference(stamp, after->first));

    return (dtBefore < dtAfter) ? before->second : after->second;
}

void StateEstimationSmoother::initialize_new_frame(
    frame_index_t id, const pair_nearby_frame_iterators_t& closestFrames)
{
    const auto stamp   = state_.stamp2frame_index.find_value(id)->second;
    const auto stamp_s = mrpt::Clock::toDouble(stamp);

    const auto closest_idx_opt = pick_closest(closestFrames, stamp);

    // Pick the data from closest frame as initial value, or 0 if none (first ever frame)
    gtsam::Pose3  pose        = gtsam::Pose3::Identity();
    gtsam::Point3 linVelocity = gtsam::Point3::Zero();
    gtsam::Point3 angVelocity = gtsam::Point3::Zero();

    // Initialize the state struct too:
    auto& newKfState = state_.last_estimated_states[id];

    if (closest_idx_opt.has_value())
    {
        const auto& kfState = state_.last_estimated_states.at(*closest_idx_opt);

        pose        = mrpt::gtsam_wrappers::toPose3(kfState.pose);
        linVelocity = {kfState.twist.vx, kfState.twist.vy, kfState.twist.vz};
        angVelocity = {kfState.twist.wx, kfState.twist.wy, kfState.twist.wz};

        // And initialize the state struct too:
        newKfState = kfState;
    }
    else
    {
        // This is the first ever frame.
        // Add weak prior factors for the system to be determinate.
        const auto priorNoise6 =
            gtsam::noiseModel::Isotropic::Sigma(6, FIRST_POSE_WEAK_PRIOR_SIGMA);

        state_.gtsam->newFactors.addPrior(T(id), pose, priorNoise6);

        const auto& tw = params_.initial_twist;
        state_.gtsam->newFactors.addPrior(
            V(id), gtsam::Vector3(tw.vx, tw.vy, tw.vz),
            gtsam::noiseModel::Isotropic::Sigma(3, params_.initial_twist_sigma_lin));

        state_.gtsam->newFactors.addPrior(
            W(id), gtsam::Vector3(tw.wx, tw.wy, tw.wz),
            gtsam::noiseModel::Isotropic::Sigma(3, params_.initial_twist_sigma_ang));

        if (params_.link_first_pose_to_reference_origin_sigma.has_value())
        {
            state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                T(id), gtsam::Pose3::Identity(),
                gtsam::noiseModel::Isotropic::Sigma(
                    6, *params_.link_first_pose_to_reference_origin_sigma));
        }
    }

    // T: Pose
    state_.gtsam->newValues.insert(T(id), pose);
    state_.gtsam->newKeyStamps[T(id)] = stamp_s;

    // V: Lin Velocity
    state_.gtsam->newValues.insert(V(id), linVelocity);
    state_.gtsam->newKeyStamps[V(id)] = stamp_s;

    // W: Ang Velocity
    state_.gtsam->newValues.insert(W(id), angVelocity);
    state_.gtsam->newKeyStamps[W(id)] = stamp_s;

    // Add planar constraints:
    if (params_.enforce_planar_motion)
    {
        const auto planar_z_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(
            PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA,
            PLANAR_Z_SIGMA));

        state_.gtsam->newFactors.addPrior(T(id), gtsam::Pose3::Identity(), planar_z_noise);
    }
}

void StateEstimationSmoother::add_kinematic_factor_between(
    const frame_index_t from, const frame_index_t to)  // NOLINT
{
    ASSERT_NOT_EQUAL_(from, to);

    // Take note of already connected frames to avoid duplications
    // --------------------------------------------------------------------
    // From => to
    {
        auto& fromKf = state_.last_estimated_states.at(from);
        if (fromKf.kinematic_links_to.count(to) != 0)
        {
            return;  // already added
        }
        fromKf.kinematic_links_to.insert(to);
    }

    // To => From
    {
        auto& toKf = state_.last_estimated_states.at(to);
        if (toKf.kinematic_links_to.count(from) != 0)
        {
            return;  // already added
        }
        toKf.kinematic_links_to.insert(from);
    }

    // Dispatch to factor generation:
    // --------------------------------------------------------------------
    const double dt = mrpt::Clock::toDouble(state_.stamp2frame_index.inverse(to)) -
                      mrpt::Clock::toDouble(state_.stamp2frame_index.inverse(from));

    switch (params_.kinematic_model)
    {
        case KinematicModel::ConstantVelocity:
        {
            AbsFactorConstVelKinematics f;
            f.from_kf   = from;
            f.to_kf     = to;
            f.deltaTime = dt;
            addFactor(f);
        }
        break;

        case KinematicModel::Tricycle:
        {
            AbsFactorTricycleKinematics f;
            f.from_kf   = from;
            f.to_kf     = to;
            f.deltaTime = dt;
            addFactor(f);
        }
        break;

        default:
            THROW_EXCEPTION("Invalid kinematic_model value");
    }
}

NavState StateEstimationSmoother::get_latest_state_and_covariance(const frame_index_t idx) const
{
    const auto& frame = state_.last_estimated_states.at(idx);

    NavState ns;

    // Pose:
    ns.pose.mean       = frame.pose;
    const auto poseCov = gtsam::Matrix6(state_.gtsam->smoother->marginalCovariance(T(idx)));
    ASSERT_(poseCov.determinant() > 0);
    ns.pose.cov_inv = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(poseCov).inverse_LLt();

    // Twist:
    ns.twist        = frame.twist;
    const auto vCov = gtsam::Matrix3(state_.gtsam->smoother->marginalCovariance(V(idx)));
    const auto wCov = gtsam::Matrix3(state_.gtsam->smoother->marginalCovariance(W(idx)));

    gtsam::Matrix6 twCov    = gtsam::Matrix6::Zero();
    twCov.block<3, 3>(0, 0) = vCov;
    twCov.block<3, 3>(3, 3) = wCov;

    ASSERT_(twCov.determinant() > 0);
    ns.twist_inv_cov = twCov.inverse();

    return ns;
}

std::optional<mrpt::poses::CPose3DPDFGaussian> StateEstimationSmoother::estimated_T_enu_to_map()
    const
{
    auto lck = mrpt::lockHelper(stateMutex_);

    auto it = state_.last_estimated_frames.find(REFERENCE_FRAME_ID);
    if (it == state_.last_estimated_frames.end())
    {
        return {};
    }
    return {it->second};
}

std::optional<mrpt::poses::CPose3DPDFGaussian>
    StateEstimationSmoother::get_estimated_T_map_to_odometry_frame(const frame_index_t idx) const
{
    ASSERT_GE_(idx, 1);
    auto lck = mrpt::lockHelper(stateMutex_);

    auto it = state_.last_estimated_frames.find(idx);
    if (it == state_.last_estimated_frames.end())
    {
        return {};
    }
    return {it->second};
}

std::optional<mrpt::poses::CPose3DPDFGaussian>
    StateEstimationSmoother::estimated_T_map_to_odometry_frame(const std::string& frame_id) const
{
    auto lck = mrpt::lockHelper(stateMutex_);

    const auto& str2id = state_.known_odom_frames.getDirectMap();
    if (auto it = str2id.find(frame_id); it != str2id.end())
    {
        return get_estimated_T_map_to_odometry_frame(it->second);
    }

    // frame not known or not estimated yet
    return {};
}

bool StateEstimationSmoother::has_converged_localization(
    mrpt::poses::CPose3DPDFGaussian& pose) const
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // We need at least some frames to have an estimate
    if (state_.last_estimated_states.empty())
    {
        return false;
    }

    // Get the latest timestamp from the state
    auto tNowOpt = state_.get_current_extrapolated_stamp();
    if (!tNowOpt)
    {
        return false;
    }

    // Find the most recent frame
    if (state_.stamp2frame_index.empty())
    {
        return false;
    }

    // Converged if we were told to estimate, and already have it:
    const bool converged = params_.estimate_geo_reference && state_.geo_reference.has_value();

    if (converged)
    {
        const auto& latestIt       = state_.stamp2frame_index.getDirectMap().rbegin();
        const auto  latestFrameIdx = latestIt->second;

        const NavState ns = get_latest_state_and_covariance(latestFrameIdx);
        pose.copyFrom(ns.pose);
    }

    return converged;
}

std::optional<mola::Georeferencing> StateEstimationSmoother::current_georeferencing() const
{
    auto lck = mrpt::lockHelper(stateMutex_);
    return state_.geo_reference;
}

void StateEstimationSmoother::publishEstimatedGeoreferencing()
{
    // Must be called with lock held
    if (!state_.tentative_geo_coord_reference.has_value())
    {
        return;
    }

    auto T_enu_map_opt = estimated_T_enu_to_map();
    if (!T_enu_map_opt.has_value())
    {
        return;
    }

    // Store as our now-fixed geo-reference
    state_.geo_reference.emplace();
    state_.geo_reference->geo_coord    = *state_.tentative_geo_coord_reference;
    state_.geo_reference->T_enu_to_map = *T_enu_map_opt;

    // Publish via MapSourceBase
    MapUpdate mu;
    mu.method          = "state_estimator";
    mu.reference_frame = params_.reference_frame_name;
    mu.timestamp       = mrpt::Clock::now();
    mu.map_name        = "georef";
    mu.georeferencing  = state_.geo_reference;

    advertiseUpdatedMap(mu);

    state_.estimated_georef_published = true;

    MRPT_LOG_THROTTLE_INFO_STREAM(
        5.0, "Published estimated geo-reference: "
                 << "lat=" << state_.geo_reference->geo_coord.lat.getAsString()
                 << ", lon=" << state_.geo_reference->geo_coord.lon.getAsString()
                 << ", T_enu_to_map=" << state_.geo_reference->T_enu_to_map.mean.asString());
}

}  // namespace mola::state_estimation_smoother
