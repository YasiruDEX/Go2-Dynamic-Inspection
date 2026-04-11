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

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <mola_sm_loop_closure/LoopClosureInterface.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/topography/data_types.h>
#include <mrpt/typemeta/TEnumType.h>

#include <set>
#include <vector>

namespace mola
{
/** Frame-to-frame GNSS-assisted loop closure engine.
 *
 * This class implements a simpler loop closure strategy than SimplemapLoopClosure:
 * 1) Uses GPS readings to optimize global frame poses
 * 2) Runs frame-to-frame ICP between loop closure candidates
 * 3) Optimizes the full graph with robust factors
 */
class FrameToFrameLoopClosure : public mola::LoopClosureInterface
{
    DEFINE_MRPT_OBJECT(FrameToFrameLoopClosure, mola)

   public:
    FrameToFrameLoopClosure();

    /** @name Main API
     * @{ */

    using frame_id_t = uint32_t;

    void initialize(const mrpt::containers::yaml& cfg) override;

    /** Find and apply loop closures in the input/output simplemap */
    void process(mrpt::maps::CSimpleMap& sm) override;

    struct Parameters
    {
        mp2p_icp::Parameters icp_parameters;

        // GNSS optimization parameters
        bool   use_gnss                          = true;
        double gnss_minimum_uncertainty_xyz      = 0.10;  // [m]
        bool   gnss_add_horizontality            = false;
        double gnss_horizontality_sigma_z        = 0.01;  // [m]
        double gnss_edges_uncertainty_multiplier = 1.0;

        // Loop closure candidate selection
        double min_distance_between_frames   = 20.0;  // [m] minimum separation for LC
        double max_distance_for_lc_candidate = 50.0;  // [m] maximum distance to consider
        size_t max_lc_candidates             = 100;  // maximum candidates to check
        size_t min_frames_between_lc         = 50;  // minimum frame separation
        size_t max_lc_optimization_rounds    = 5;  // maximum LC+optimization rounds to run

        /** Loop closure candidate selection strategy */
        enum class CandidateSelectionStrategy : uint8_t
        {
            PROXIMITY_ONLY      = 0,  ///< Simple method: score = 1/(1+distance)
            DISTANCE_STRATIFIED = 1,  ///< Stratified sampling across distance bins
            MULTI_OBJECTIVE     = 2,  ///< Complex multi-criteria scoring
        };

        /** Active candidate selection strategy */
        CandidateSelectionStrategy lc_candidate_strategy =
            CandidateSelectionStrategy::DISTANCE_STRATIFIED;

        /** Number of distance bins for DISTANCE_STRATIFIED strategy
         * The valid distance range [min_distance_between_frames,
         * max_distance_for_lc_candidate] is divided into this many bins,
         * and candidates are sampled proportionally from each bin.
         * Typical values: 3-7
         */
        size_t lc_distance_bins = 5;

        /** Weight for proximity in multi-objective scoring
         * How much to favor spatially close candidates.
         * Valid range: [0.0, 1.0]
         */
        double lc_weight_proximity = 0.40;

        /** Weight for frame separation in multi-objective scoring
         * How much to favor temporally distant candidates (large frame index gap).
         * Valid range: [0.0, 1.0]
         */
        double lc_weight_frame_separation = 0.25;

        /** Weight for distance diversity in multi-objective scoring
         * How much to penalize candidates with similar distances to already selected ones.
         * Valid range: [0.0, 1.0]
         */
        double lc_weight_diversity = 0.20;

        /** Weight for geometric coverage in multi-objective scoring
         * How much to favor candidates that cover different parts of the trajectory.
         * Valid range: [0.0, 1.0]
         */
        double lc_weight_coverage = 0.15;

        /** Enable detailed logging of candidate selection process
         * When true, logs information about each distance bin, selection statistics,
         * and distribution of selected candidates.
         */
        bool lc_verbose_candidate_selection = false;

        // ICP parameters
        double      min_icp_goodness              = 0.50;
        double      icp_edge_robust_param         = 5.0;
        double      icp_edge_additional_noise_xyz = 0.02;  // [m]
        double      icp_edge_additional_noise_ang = 0.1;  // [deg]
        std::string threshold_sigma_initial       = "0.10";
        std::string threshold_sigma_final         = "0.05";

        // Odometry edge parameters
        double input_odometry_noise_xyz = 0.01;  // [m]
        double input_odometry_noise_ang = 0.1;  // [deg]

        // Optimization parameters
        double largest_delta_for_reconsider = 15.0;  // [m] re-check LCs if change > this

        // Sensor parameters
        double max_sensor_range = 100.0;  // [m]

        // Output and profiling
        bool        profiler_enabled               = true;
        bool        save_trajectory_files          = true;
        bool        save_trajectory_files_with_cov = false;
        std::string debug_files_prefix             = "f2f_lc_";

        // 3D scene visualization output
        bool  save_3d_scene_files       = false;
        float scene_path_line_width     = 2.0f;
        float scene_lc_line_width       = 4.0f;
        float scene_path_color_r        = 0.0f;
        float scene_path_color_g        = 0.0f;
        float scene_path_color_b        = 1.0f;  // blue
        float scene_path_color_a        = 0.7f;
        float scene_lc_color_r          = 1.0f;  // red
        float scene_lc_color_g          = 0.0f;
        float scene_lc_color_b          = 0.0f;
        float scene_lc_color_a          = 0.8f;
        float scene_keyframe_point_size = 7.0f;
    };

    Parameters params_;

    /** @} */

   private:
    struct PerThreadState
    {
        std::mutex mtx;

        mp2p_icp::ParameterSource parameter_source;
        mp2p_icp::ICP::Ptr        icp;

        // For processing observations
        mp2p_icp_filters::GeneratorSet   obs_generators;
        mp2p_icp_filters::FilterPipeline pc_filter;

        mrpt::expr::CRuntimeCompiledExpression expr_threshold_sigma_initial;
        mrpt::expr::CRuntimeCompiledExpression expr_threshold_sigma_final;
    };

    struct State
    {
        bool initialized = false;

        const mrpt::maps::CSimpleMap* sm = nullptr;

        // Per-thread ICP instances
        std::vector<PerThreadState> perThreadState_{
            std::max(1u, std::thread::hardware_concurrency())};

        // GNSS reference coordinate
        std::optional<mrpt::topography::TGeodeticCoords> globalGeoRef;

        // GTSAM graph and values
        gtsam::Values                   graphValues;
        gtsam::NonlinearFactorGraph     graphFG;
        std::optional<gtsam::Marginals> graphMarginals;

        [[nodiscard]] mrpt::poses::CPose3D get_pose(frame_id_t id) const;

        /// Cov in MRPT order: xyz yaw pitch roll
        [[nodiscard]] mrpt::math::CMatrixDouble66 get_pose_cov(frame_id_t id) const;
    };

    State state_;

    mrpt::system::CTimeLogger profiler_{true, "frame_to_frame_lc"};
    mrpt::WorkerThreadsPool   threads_{state_.perThreadState_.size()};

    // Private methods
    mrpt::poses::CPose3D frame_pose_in_simplemap(frame_id_t frameId) const;

    /** Generate point cloud from a frame's observations */
    mp2p_icp::metric_map_t::Ptr generate_frame_pointcloud(frame_id_t frameId, size_t threadIdx);

    /** Build initial graph with odometry and GNSS factors */
    void build_initial_graph();

    /** Add GNSS factors to the graph */
    void add_gnss_factors();

    struct LoopCandidate
    {
        frame_id_t frame_i  = 0;
        frame_id_t frame_j  = 0;
        double     distance = 0.0;  // estimated distance between frames
        double     score    = 0.0;  // candidate quality score
    };

    /** Find potential loop closure candidates */
    std::vector<LoopCandidate> find_loop_candidates(
        const std::set<std::pair<frame_id_t, frame_id_t>>& alreadyChecked) const;

    /** Process a single loop closure candidate with ICP */
    bool process_loop_candidate(const LoopCandidate& lc);

    /** Optimize the graph and return the largest pose change */
    double optimize_graph();

    /** Save trajectory to TUM format file */
    void save_trajectory_as_tum(const std::string& filename, bool saveCovariancesToo = false) const;

    /** Save 3D scene visualization files (final optimized poses) */
    void save_3d_scene_files() const;

    /** Save 3D scene visualization files (original poses before LC) */
    void save_3d_scene_initial_files() const;

    /** Update dynamic variables for ICP pipeline */
    void update_dynamic_variables(frame_id_t frameId, size_t threadIdx);

    /** Accepted loop closure edges (for 3D scene output) */
    std::vector<std::pair<frame_id_t, frame_id_t>> accepted_lc_edges_;
};

}  // namespace mola

// Enum type machinery for CandidateSelectionStrategy:
MRPT_ENUM_TYPE_BEGIN_NAMESPACE(
    mola, mola::FrameToFrameLoopClosure::Parameters::CandidateSelectionStrategy)
MRPT_FILL_ENUM_CUSTOM_NAME(
    FrameToFrameLoopClosure::Parameters::CandidateSelectionStrategy::PROXIMITY_ONLY,
    "PROXIMITY_ONLY");
MRPT_FILL_ENUM_CUSTOM_NAME(
    FrameToFrameLoopClosure::Parameters::CandidateSelectionStrategy::DISTANCE_STRATIFIED,
    "DISTANCE_STRATIFIED");
MRPT_FILL_ENUM_CUSTOM_NAME(
    FrameToFrameLoopClosure::Parameters::CandidateSelectionStrategy::MULTI_OBJECTIVE,
    "MULTI_OBJECTIVE");
MRPT_ENUM_TYPE_END()
