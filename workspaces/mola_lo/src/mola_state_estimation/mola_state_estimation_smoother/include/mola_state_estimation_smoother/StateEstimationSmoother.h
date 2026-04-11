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
 * @file   StateEstimationSmoother.h
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

// this package:
#include <mola_gtsam_factors/id.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_kernel/version.h>
#include <mola_state_estimation_smoother/Parameters.h>
#include <mola_state_estimation_smoother/RegexCache.h>

// MOLA:
#include <mola_imu_preintegration/ImuIntegrator.h>

// MRPT:
#include <mrpt/containers/bimap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

// std:
#include <mutex>
#include <optional>
#include <set>

namespace mola::state_estimation_smoother
{
/** Sliding window Factor-graph data fusion for odometry, IMU, GNSS, and SE(3)
 * pose/twist estimations.
 *
 * Frame conventions:
 * - There is a frame of reference for each source of odometry, e.g.
 *   there may be one for LiDAR-odometry, another for visual-odometry, or
 *   wheels-based odometry, etc. Each such frame is referenced with a "frame
 *   name" (an arbitrary string).
 *
 * - Internally, this class uses the {utm}, {enu}, and {map} frames. Refer to
 *   the frame diagrams [here](https://docs.mola-slam.org/latest/mola_state_estimators.html).
 *
 * - The name for the reference frame (Default: `"map"`) and the robot/vehicle (`"base_link"`)
 *   can be changed from the parameters (e.g. the config yaml file).
 *
 * - This package DOES NOT follow the [ROS REP 105](https://www.ros.org/reps/rep-0105.html)
 *   specifications in the sense that `/tf` from `{map} → {odom}` are not published.
 *   Instead, it directly emits `{map} → {base_link}` from the fusion of all available data.
 *
 * - Publishing the vehicle pose in a timely manner uses "params.reference_frame_name" as
 *   reference frame.
 *
 * - IMU readings are, by definition, given in the local robot body frame, although
 *   they can have a relative transformation between the vehicle and sensor.
 *
 * Main API methods and frame conventions:
 * - `estimated_navstate()`: Output estimations can be requested in any of the
 *      existing frames of reference.
 * - `fuse_pose()`: Can be used to integrate information from any "odometry" or
 *     "localization" input, as mentioned above.
 * - `fuse_gnss()`: Integrate GNSS observations, to help with localization in geo-referenced maps,
 *     or to automatically find-out the geo-referencing of a map.
 * - `fuse_imu()`: Used to help with (1) global azimuth in geo-referenced maps, (2) vertical
 *     direction from accelerometer, (3) angular velocity from gyroscope.
 *
 * Usage:
 * - (1) Call initialize() to set the required parameters.
 * - (2) Integrate measurements with `fuse_*()` methods. Each CObservation
 *       class includes a `timestamp` field which is used to estimate the
 *       trajectory.
 * - (3) Repeat (2) as needed.
 * - (4) Read the estimation up to any nearby moment in time with
 *       estimated_navstate()
 *
 * Old observations are automatically removed.
 *
 * A constant SE(3) velocity model is internally used, without any
 * particular assumptions on the vehicle kinematics.
 *
 * For more theoretical descriptions, see:
 * https://docs.mola-slam.org/latest/mola_state_estimators.html
 *
 * \ingroup mola_state_estimation_grp
 */
class StateEstimationSmoother : public mola::NavStateFilter,
                                public mola::LocalizationSourceBase,
                                public mola::MapSourceBase

{
    DEFINE_MRPT_OBJECT(StateEstimationSmoother, mola::state_estimation_smoother)
   private:
    class AbsFactorConstVelKinematics;  // Forward decls.
    class AbsFactorTricycleKinematics;

   public:
    StateEstimationSmoother();

    // Make not copiable due to the pimpl gtsam state.
    StateEstimationSmoother(const StateEstimationSmoother&)            = delete;
    StateEstimationSmoother(StateEstimationSmoother&&)                 = delete;
    StateEstimationSmoother& operator=(const StateEstimationSmoother&) = delete;
    StateEstimationSmoother& operator=(StateEstimationSmoother&&)      = delete;
    ~StateEstimationSmoother()                                         = default;

    /** \name Main API
     *  @{ */

    /** Parameters can only be set via initialize(), then read-only accesses through this method. */
    const Parameters& parameters() { return params_; }

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const mrpt::containers::yaml& cfg) override;

    void spinOnce() override;

    /** Resets the estimator state to an initial state */
    void reset() override;

    /** Integrates new SE(3) pose odometry estimation of the vehicle wrt frame_id
     */
    void fuse_pose(
        const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string& frame_id) override;

    /** Integrates new wheels-based odometry observations into the estimator.
     *  This is a convenience method that internally ends up calling
     *  fuse_pose(), but computing the uncertainty of odometry increments
     *  according to a given motion model.
     */
    void fuse_odometry(
        const mrpt::obs::CObservationOdometry& odom,
        const std::string&                     odomName = "odom_wheels") override;

    /** Integrates new IMU observations into the estimator */
    void fuse_imu(const mrpt::obs::CObservationIMU& imu) override;

    /** Integrates new GNSS observations into the estimator */
    void fuse_gnss(const mrpt::obs::CObservationGPS& gps) override;

    /** Integrates new twist estimation (in the odom frame) */
    void fuse_twist(
        const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
        const mrpt::math::CMatrixDouble66& twistCov) override;

    /** Computes the estimated vehicle state at a given timestep using the
     * observations in the time window. A std::nullopt is returned if there is
     * no valid observations yet, or if requested a timestamp out of the model
     * validity time window (e.g. too far in the future to be trustful).
     */
    [[nodiscard]] std::optional<NavState> estimated_navstate(
        const mrpt::Clock::time_point& timestamp, const std::string& frame_id) override;

    /// Returns a list of known odometry frame_ids:
    [[nodiscard]] auto known_odometry_frame_ids() -> std::set<std::string>;

    /// Gets the latest estimated transform of T_enu_to_map
    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> estimated_T_enu_to_map() const;

    /// Gets the latest estimated transform of "T_map_to_odometry_frame_i", by frame ID name.
    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> estimated_T_map_to_odometry_frame(
        const std::string& frame_id) const;

    /// Implements NavStateFilter::has_converged_localization
    [[nodiscard]] bool has_converged_localization(mrpt::poses::CPose3DPDFGaussian& pose) const
#if MOLA_VERSION_CHECK(2, 5, 0)
        override
#endif
        ;

    /// Returns the current georeferencing, if available
    [[nodiscard]] std::optional<mola::Georeferencing> current_georeferencing() const;

    /** @} */

    // Implementation of RawDataConsumer
    void onNewObservation(const CObservation::ConstPtr& o) override;

   private:
    Parameters params_;

    // everything related to gtsam is hidden in the public API via pimpl
    // to reduce compilation dependencies, and build time and memory usage.
    struct GtsamImpl;

    using odometry_frameid_t = uint8_t;
    using frame_index_t      = uint32_t;

    struct FrameState
    {
        mrpt::poses::CPose3D    pose;  //!< in the reference frame
        mrpt::math::TTwist3D    twist;  //!< in the local frame of reference
        std::set<frame_index_t> kinematic_links_to;
    };

    // Accesses to this struct values in state_ must be protected by stateMutex_
    struct State
    {
        State();

        mrpt::pimpl<GtsamImpl> gtsam;

        /// The next numeric ID to assign to a new frame, for usage in GTSAM symbols P(i), v(i)...
        frame_index_t next_frame_index = 0;

        /// A bimap of timestamps <=> frame indices. Updated by
        mrpt::containers::bimap<mrpt::Clock::time_point, frame_index_t> stamp2frame_index;

        /// A bimap of known odometry "frame_id" <=> "numeric IDs":
        mrpt::containers::bimap<std::string, odometry_frameid_t> known_odom_frames;

        /// The latest values from the estimator; updated in process_pending_gtsam_updates()
        std::map<frame_index_t, FrameState> last_estimated_states;

        /// The latest values from the estimator; updated in process_pending_gtsam_updates()
        std::map<odometry_frameid_t, mrpt::poses::CPose3DPDFGaussian> last_estimated_frames;

        /** For real-time mode operation (not offline): returns the current extrapolated stamp,
         *  by adding the difference between the last observation wallclock time and now to the
         *  last observation timestamp.
         */
        std::optional<mrpt::Clock::time_point> get_current_extrapolated_stamp() const
        {
            if (!last_observation_stamp)
            {
                return {};
            }
            return mrpt::Clock::fromDouble(
                (mrpt::Clock::nowDouble() -
                 mrpt::Clock::toDouble(last_observation_wallclock_stamp)) +
                mrpt::Clock::toDouble(*last_observation_stamp));
        }

        std::optional<mrpt::Clock::time_point> last_observation_stamp;
        mrpt::Clock::time_point                last_observation_wallclock_stamp;

        std::optional<mrpt::poses::CPose2D> last_wheels_odometry;
        std::optional<std::string>          last_wheels_odometry_name;

        /** Refer to Parameters for possible sources of this.
         * Anyways: this will always hold either the estimated or the fixed (externally set)
         * georeferencing parameters.
         * When this is still empty, it means we are still waiting for someone external to
         * send us the georeferencing data, or our internal estimator didn't obtained a quality
         * estimation yet.
         */
        std::optional<mola::Georeferencing> geo_reference;

        /// Will be populated with the first GNSS coords when in active estimation mode.
        std::optional<mrpt::topography::TGeodeticCoords> tentative_geo_coord_reference;

        /// Flag to track if we've already published the estimated geo-ref
        bool estimated_georef_published = false;

        // To be built from parameters strings when changed.
        RegexCache do_process_imu_labels_re;
        RegexCache do_process_odometry_labels_re;
        RegexCache do_process_gnss_labels_re;
    };

    State                state_;
    std::recursive_mutex stateMutex_;

    /// Creates a new frame index for timestamp t, or returns the existing one if close enough.
    /// This also is in charge of the complex task of finding nearby existing frames and adding the
    /// kinematic factors to ensure smooth motion estimation.
    [[nodiscard]] frame_index_t create_or_get_keyframe_by_timestamp(
        const mrpt::Clock::time_point& t,
        const std::optional<double>&   overrideCloseEnough = std::nullopt);

    /// Creates or returns the existing ID, for an odometry frame_id:
    [[nodiscard]] odometry_frameid_t add_or_get_odom_frame_id(const std::string& frame_id_name);

    /// Adds new factors to the smoother, optimizes it, and saves the variable values into
    /// state_.last_estimated_state
    void process_pending_gtsam_updates();

    /// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
    void addFactor(const AbsFactorConstVelKinematics& f);
    void addFactor(const AbsFactorTricycleKinematics& f);

    /// Delete out-of-window entries in stamp2frame_index and last_estimated_state
    void delete_too_old_entries();

    void publishEstimatedGeoreferencing();

    using pair_nearby_frame_iterators_t = std::pair<
        std::map<mrpt::Clock::time_point, frame_index_t>::const_iterator,
        std::map<mrpt::Clock::time_point, frame_index_t>::const_iterator>;

    [[nodiscard]] pair_nearby_frame_iterators_t find_before_after(
        const mrpt::Clock::time_point& t, bool allow_exact_match);

    void initialize_new_frame(frame_index_t id, const pair_nearby_frame_iterators_t& closestFrames);

    [[nodiscard]] std::optional<frame_index_t> pick_closest(
        const pair_nearby_frame_iterators_t& closestFrames,
        const mrpt::Clock::time_point&       stamp) const;

    void add_kinematic_factor_between(const frame_index_t from, const frame_index_t to);

    /// Gets the latest state of a pose wrt the reference frame ("map")
    [[nodiscard]] NavState get_latest_state_and_covariance(const frame_index_t idx) const;

    /// Gets the latest estimated transform of "T_map_to_odometry_frame_i", by frame ID name.
    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian>
        get_estimated_T_map_to_odometry_frame(const frame_index_t idx) const;

    /** Abstract representation of a constant-velocity kinematic motion model factor
     * between two key frames.
     */
    class AbsFactorConstVelKinematics
    {
       public:
        AbsFactorConstVelKinematics() = default;

        /** Creates relative pose constraint of KF `to` as seem from `from`. */
        AbsFactorConstVelKinematics(id_t kf_from, id_t kf_to, double delta_time)  // NOLINT
            : from_kf(kf_from), to_kf(kf_to), deltaTime(delta_time)
        {
        }

        id_t from_kf = INVALID_ID, to_kf = INVALID_ID;

        /** Elapsed time between "from_kf" and "to_kf" [seconds] */
        double deltaTime = .0;
    };

    /** Abstract representation of a constant-velocity tricycle kinematic motion
     * model factor between two key frames.
     */
    class AbsFactorTricycleKinematics
    {
       public:
        AbsFactorTricycleKinematics() = default;

        /** Creates relative pose constraint of KF `to` as seem from `from`. */
        AbsFactorTricycleKinematics(id_t kf_from, id_t kf_to, double delta_time)  // NOLINT
            : from_kf(kf_from), to_kf(kf_to), deltaTime(delta_time)
        {
        }

        id_t from_kf = INVALID_ID, to_kf = INVALID_ID;

        /** Elapsed time between "from_kf" and "to_kf" [seconds] */
        double deltaTime = .0;
    };

    VizInterface::Ptr visualizer_;
};

}  // namespace mola::state_estimation_smoother
