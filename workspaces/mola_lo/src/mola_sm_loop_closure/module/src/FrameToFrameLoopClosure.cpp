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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_gtsam_factors/FactorGnssEnu.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>
#include <mola_sm_loop_closure/FrameToFrameLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/system/filesystem.h>

#include <cmath>
#include <numeric>

using namespace mola;

IMPLEMENTS_SERIALIZABLE(FrameToFrameLoopClosure, LoopClosureInterface, mola)

namespace
{
const bool PRINT_LC_SCORES = mrpt::get_env<bool>("PRINT_LC_SCORES", false);
const bool SAVE_ICP_LOGS   = mrpt::get_env<bool>("SAVE_ICP_LOGS", false);

bool frame_has_mapping_observations(const mrpt::obs::CSensoryFrame& sf)
{
    if (sf.empty())
    {
        return false;
    }

    if (sf.getObservationByClass<mrpt::obs::CObservationPointCloud>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservation2DRangeScan>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>())
    {
        return true;
    }
    if (sf.getObservationByClass<mrpt::obs::CObservationVelodyneScan>())
    {
        return true;
    }

    return false;
}

/**
 * Compute score using original proximity-only strategy
 */
double score_proximity_only(double distance) { return 1.0 / (1.0 + distance); }

/**
 * Compute score for distance-stratified strategy
 * Combines proximity with frame separation
 */
double score_stratified(
    double distance, double minDist, double maxDist, size_t frameI, size_t frameJ,
    size_t totalFrames)
{
    const double distRange = maxDist - minDist;
    const double normDist  = (distance - minDist) / distRange;

    // Softer proximity score
    const double proximityScore = std::sqrt(1.0 - normDist);

    // Frame separation bonus
    const double frameSep        = static_cast<double>(frameJ - frameI);
    const double maxFrameSep     = static_cast<double>(totalFrames);
    const double separationScore = frameSep / maxFrameSep;

    return 0.6 * proximityScore + 0.4 * separationScore;
}

/**
 * Compute score using multi-objective strategy
 */
double score_multi_objective(
    double distance, [[maybe_unused]] double minDist, [[maybe_unused]] double maxDist,
    size_t frameI, size_t frameJ, size_t totalFrames, const std::vector<double>& selectedDistances,
    double wProx, double wSep, double wDiv, double wCov)
{
    // 1. Proximity score
    const double proximityScore = 1.0 / (1.0 + distance);

    // 2. Frame separation score
    const auto   frameSep        = static_cast<double>(frameJ - frameI);
    const auto   maxFrameSep     = static_cast<double>(totalFrames);
    const double separationScore = frameSep / maxFrameSep;

    // 3. Distance diversity score
    double diversityScore = 1.0;
    for (const auto existingDist : selectedDistances)
    {
        const double distDiff = std::abs(distance - existingDist);
        const double penalty  = std::exp(-distDiff / 5.0);  // 5m characteristic scale
        diversityScore *= (1.0 - 0.3 * penalty);
    }

    // 4. Geometric coverage score (trajectory mid-point coverage)
    const double midPoint      = static_cast<double>(frameI + frameJ) / 2.0;
    const double coverageScore = std::abs(std::sin(M_PI * midPoint / maxFrameSep));

    // Normalize weights (in case they don't sum to 1.0)
    const double wSum = wProx + wSep + wDiv + wCov;
    const double w1   = wProx / wSum;
    const double w2   = wSep / wSum;
    const double w3   = wDiv / wSum;
    const double w4   = wCov / wSum;

    return w1 * proximityScore + w2 * separationScore + w3 * diversityScore + w4 * coverageScore;
}

std::string first_n_lines(const std::string& input, std::size_t n)
{
    if (n == 0)
    {
        return {};
    }

    std::size_t pos   = 0;
    std::size_t lines = 0;

    while (lines < n)
    {
        pos = input.find('\n', pos);
        if (pos == std::string::npos)
        {
            // Fewer than n lines: return entire string
            return input;
        }
        ++pos;  // move past '\n'
        ++lines;
    }

    return input.substr(0, pos);
}

}  // namespace

FrameToFrameLoopClosure::FrameToFrameLoopClosure()
{
    mrpt::system::COutputLogger::setLoggerName("FrameToFrameLoopClosure");
    threads_.name("f2f_icp_threads");
}

void FrameToFrameLoopClosure::initialize(const mrpt::containers::yaml& c)
{
    MRPT_TRY_START

    const auto cfg = c["params"];

    // Load parameters
    YAML_LOAD_OPT(params_, use_gnss, bool);
    YAML_LOAD_OPT(params_, gnss_minimum_uncertainty_xyz, double);
    YAML_LOAD_OPT(params_, gnss_add_horizontality, bool);
    YAML_LOAD_OPT(params_, gnss_horizontality_sigma_z, double);
    YAML_LOAD_OPT(params_, gnss_edges_uncertainty_multiplier, double);

    YAML_LOAD_OPT(params_, min_distance_between_frames, double);
    YAML_LOAD_OPT(params_, max_distance_for_lc_candidate, double);
    YAML_LOAD_OPT(params_, max_lc_candidates, size_t);
    YAML_LOAD_OPT(params_, min_frames_between_lc, size_t);
    YAML_LOAD_OPT(params_, max_lc_optimization_rounds, size_t);

    if (params_.min_frames_between_lc == 0)
    {
        MRPT_LOG_WARN("min_frames_between_lc=0 is invalid; clamping to 1.");
        params_.min_frames_between_lc = 1;
    }

    YAML_LOAD_OPT(params_, lc_distance_bins, size_t);
    YAML_LOAD_OPT(params_, lc_weight_proximity, double);
    YAML_LOAD_OPT(params_, lc_weight_frame_separation, double);
    YAML_LOAD_OPT(params_, lc_weight_diversity, double);
    YAML_LOAD_OPT(params_, lc_weight_coverage, double);
    YAML_LOAD_OPT(params_, lc_verbose_candidate_selection, bool);

    // Load enum with string-to-enum conversion
    if (cfg.has("lc_candidate_strategy"))
    {
        params_.lc_candidate_strategy =
            mrpt::typemeta::TEnumType<Parameters::CandidateSelectionStrategy>::name2value(
                cfg["lc_candidate_strategy"].as<std::string>());
    }

    // Validate parameters
    if (params_.lc_distance_bins == 0)
    {
        MRPT_LOG_WARN("lc_distance_bins=0 is invalid; clamping to 1.");
        params_.lc_distance_bins = 1;
    }

    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, icp_edge_robust_param, double);
    YAML_LOAD_OPT(params_, icp_edge_additional_noise_xyz, double);
    YAML_LOAD_OPT(params_, icp_edge_additional_noise_ang, double);
    YAML_LOAD_OPT(params_, threshold_sigma_initial, std::string);
    YAML_LOAD_OPT(params_, threshold_sigma_final, std::string);

    YAML_LOAD_OPT(params_, input_odometry_noise_xyz, double);
    YAML_LOAD_OPT(params_, input_odometry_noise_ang, double);

    YAML_LOAD_OPT(params_, largest_delta_for_reconsider, double);
    YAML_LOAD_OPT(params_, max_sensor_range, double);

    YAML_LOAD_OPT(params_, profiler_enabled, bool);
    YAML_LOAD_OPT(params_, save_trajectory_files, bool);
    YAML_LOAD_OPT(params_, save_trajectory_files_with_cov, bool);
    YAML_LOAD_OPT(params_, debug_files_prefix, std::string);

    YAML_LOAD_OPT(params_, save_3d_scene_files, bool);
    YAML_LOAD_OPT(params_, scene_path_line_width, float);
    YAML_LOAD_OPT(params_, scene_lc_line_width, float);
    YAML_LOAD_OPT(params_, scene_path_color_r, float);
    YAML_LOAD_OPT(params_, scene_path_color_g, float);
    YAML_LOAD_OPT(params_, scene_path_color_b, float);
    YAML_LOAD_OPT(params_, scene_path_color_a, float);
    YAML_LOAD_OPT(params_, scene_lc_color_r, float);
    YAML_LOAD_OPT(params_, scene_lc_color_g, float);
    YAML_LOAD_OPT(params_, scene_lc_color_b, float);
    YAML_LOAD_OPT(params_, scene_lc_color_a, float);
    YAML_LOAD_OPT(params_, scene_keyframe_point_size, float);

    profiler_.enable(params_.profiler_enabled);

    // Initialize ICP pipelines for each thread
    ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings");

    for (auto& pts : state_.perThreadState_)
    {
        const auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(c["icp_settings"]);
        pts.icp                     = icp;
        params_.icp_parameters      = icpParams;

        pts.icp->attachToParameterSource(pts.parameter_source);

        // Observation generators
        if (c.has("observations_generator") && !c["observations_generator"].isNullNode())
        {
            pts.obs_generators =
                mp2p_icp_filters::generators_from_yaml(c["observations_generator"]);
        }
        else
        {
            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            pts.obs_generators.push_back(defaultGen);
        }
        mp2p_icp::AttachToParameterSource(pts.obs_generators, pts.parameter_source);

        // Observation filters
        if (c.has("observations_filter"))
        {
            pts.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(c["observations_filter"]);
            mp2p_icp::AttachToParameterSource(pts.pc_filter, pts.parameter_source);
        }
    }

    state_.initialized = true;

    MRPT_TRY_END
}

void FrameToFrameLoopClosure::process(mrpt::maps::CSimpleMap& sm)
{
    using namespace std::string_literals;

    ASSERT_(state_.initialized);
    state_.sm = &sm;
    accepted_lc_edges_.clear();

    MRPT_LOG_INFO_STREAM("Processing simplemap with " << sm.size() << " frames");

    // Build initial graph with odometry edges
    build_initial_graph();

    if (params_.save_trajectory_files)
    {
        optimize_graph();

        save_trajectory_as_tum(
            params_.debug_files_prefix + "initial.tum"s, params_.save_trajectory_files_with_cov);
    }

    // Add GNSS factors if available
    if (params_.use_gnss)
    {
        add_gnss_factors();

        // Initial optimization with GNSS

        MRPT_LOG_INFO("Running initial GNSS optimization...");
        optimize_graph();

        if (params_.save_trajectory_files)
        {
            save_trajectory_as_tum(
                params_.debug_files_prefix + "after_gnss.tum"s,
                params_.save_trajectory_files_with_cov);
        }
    }

    if (params_.save_3d_scene_files)
    {
        save_3d_scene_initial_files();
    }

    // Loop closure detection and optimization
    size_t                                      accepted_lcs = 0;
    std::set<std::pair<frame_id_t, frame_id_t>> alreadyChecked;

    for (size_t lcRound = 0; lcRound < params_.max_lc_optimization_rounds; lcRound++)
    {
        size_t checkedCount   = 0;
        bool   anyGraphChange = false;

        auto candidates = find_loop_candidates(alreadyChecked);

        MRPT_LOG_INFO_STREAM("Found " << candidates.size() << " loop closure candidates");

        const auto frameGroup = static_cast<double>(params_.min_frames_between_lc);

        for (const auto& lc : candidates)
        {
            // Decimate the frame IDs so we are effectively counting "blocks" of frames for what
            // concerns already-checked:
            const auto frameGroup_i = mrpt::round(static_cast<double>(lc.frame_i) / frameGroup);
            const auto frameGroup_j = mrpt::round(static_cast<double>(lc.frame_j) / frameGroup);

            const auto IDs = std::make_pair(
                std::min<frame_id_t>(frameGroup_i, frameGroup_j),
                std::max<frame_id_t>(frameGroup_i, frameGroup_j));

            if (alreadyChecked.count(IDs) != 0)
            {
                continue;
            }

            alreadyChecked.insert(IDs);
            checkedCount++;

            const bool accepted = process_loop_candidate(lc);
            if (accepted)
            {
                anyGraphChange = true;
                accepted_lcs++;
            }
        }

        if (checkedCount == 0)
        {
            break;  // No new candidates
        }

        if (anyGraphChange)
        {
            const double largestDelta = optimize_graph();

            if (largestDelta > params_.largest_delta_for_reconsider)
            {
                MRPT_LOG_INFO_STREAM(
                    "Large pose change detected (" << largestDelta
                                                   << "m), reconsidering all candidates");
                alreadyChecked.clear();
            }
        }
    }

    MRPT_LOG_INFO_STREAM("Total accepted loop closures: " << accepted_lcs);

    if (params_.save_trajectory_files)
    {
        save_trajectory_as_tum(params_.debug_files_prefix + "final.tum"s);
    }

    if (params_.save_3d_scene_files)
    {
        save_3d_scene_files();
    }

    // Update simplemap with optimized poses
    mrpt::maps::CSimpleMap outSM;
    for (size_t id = 0; id < sm.size(); id++)
    {
        auto& [oldPose, sf, twist] = sm.get(id);

        const auto newPose = mrpt::poses::CPose3DPDFGaussian::Create();
        newPose->mean      = state_.get_pose(id);
        newPose->cov.setIdentity();

        outSM.insert(newPose, sf, twist);
    }

    sm = outSM;  // TODO: Make CSimpleMap move constructible
}

void FrameToFrameLoopClosure::build_initial_graph()
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "build_initial_graph");

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    // Add all frame poses to values
    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto pose_i = frame_pose_in_simplemap(i);
        state_.graphValues.insert(X(i), mrpt::gtsam_wrappers::toPose3(pose_i));
    }

    // Add prior on first frame: very weak, so GNSS can override it as needed.
    // if not using GNSS, let X(0) be anchored.
    const double priorSigma = params_.use_gnss ? 1e+2 : 1e-2;

    const auto pose0      = frame_pose_in_simplemap(0);
    auto       priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, priorSigma);

    state_.graphFG.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(0), mrpt::gtsam_wrappers::toPose3(pose0), priorNoise);

    // Add odometry edges between consecutive frames
    for (size_t i = 1; i < sm.size(); i++)
    {
        const auto pose_i   = frame_pose_in_simplemap(i);
        const auto pose_im1 = frame_pose_in_simplemap(i - 1);

        const auto relPose   = pose_i - pose_im1;
        const auto deltaPose = mrpt::gtsam_wrappers::toPose3(relPose);

        gtsam::Vector6 sigmas;
        sigmas << mrpt::DEG2RAD(params_.input_odometry_noise_ang),
            mrpt::DEG2RAD(params_.input_odometry_noise_ang),
            mrpt::DEG2RAD(params_.input_odometry_noise_ang), params_.input_odometry_noise_xyz,
            params_.input_odometry_noise_xyz, params_.input_odometry_noise_xyz;

        auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

#if GTSAM_USES_BOOST
        auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);
#else
        auto factor = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);
#endif
        state_.graphFG += factor;
    }

    MRPT_LOG_INFO_STREAM("Built initial graph with " << sm.size() << " frames");
}

void FrameToFrameLoopClosure::add_gnss_factors()
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "add_gnss_factors");

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    // Extract GNSS frames
    AddGNSSFactorParams gpsParams;
    gpsParams.minimumUncertaintyXYZ       = params_.gnss_minimum_uncertainty_xyz;
    gpsParams.addHorizontalityConstraints = params_.gnss_add_horizontality;
    gpsParams.horizontalitySigmaZ         = params_.gnss_horizontality_sigma_z;

    const auto gnssFrames = extract_gnss_frames_from_sm(sm, state_.globalGeoRef);

    if (gnssFrames.frames.empty())
    {
        MRPT_LOG_WARN("No valid GNSS observations found");
        return;
    }

    if (!state_.globalGeoRef.has_value())
    {
        state_.globalGeoRef = gnssFrames.refCoord;
    }

    MRPT_LOG_INFO_STREAM("Adding " << gnssFrames.frames.size() << " GNSS factors");

    // Add GNSS factors for each frame
    for (const auto& gf : gnssFrames.frames)
    {
        // Find which frame this corresponds to
        frame_id_t frameId = 0;
        bool       found   = false;

        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto& [pose, sf, twist] = sm.get(i);

            for (const auto& obs : *sf)
            {
                if (obs.get() == gf.obs.get())
                {
                    frameId = i;
                    found   = true;
                    break;
                }
            }
            if (found)
            {
                break;
            }
        }

        if (!found)
        {
            continue;
        }

        auto noiseOrg = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(gf.sigma_E, gf.sigma_N, gf.sigma_U)
                .array()
                .max(params_.gnss_minimum_uncertainty_xyz) *
            params_.gnss_edges_uncertainty_multiplier);

        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(gf.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(gf.obs->sensorPose.translation());

        state_.graphFG.emplace_shared<mola::factors::FactorGnssEnu>(
            X(frameId), sensorPointOnVeh, observedENU, robustNoise);
    }
}

auto FrameToFrameLoopClosure::
    find_loop_candidates(  // NOLINT(readability-function-cognitive-complexity)
        const std::set<std::pair<frame_id_t, frame_id_t>>& alreadyChecked) const
    -> std::vector<FrameToFrameLoopClosure::LoopCandidate>
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "find_loop_candidates");

    ASSERT_(state_.sm);
    const auto& sm = *state_.sm;

    const auto   frameGroup = static_cast<double>(params_.min_frames_between_lc);
    const double minDist    = params_.min_distance_between_frames;
    const double maxDist    = params_.max_distance_for_lc_candidate;

    // For multi-objective strategy: track selected distances for diversity scoring
    std::vector<double> selectedDistances;

    // Determine if we need distance binning
    const bool useStratification =
        (params_.lc_candidate_strategy ==
         Parameters::CandidateSelectionStrategy::DISTANCE_STRATIFIED);

    // Setup distance bins if using stratified strategy
    std::vector<std::vector<LoopCandidate>> binnedCandidates;
    double                                  binWidth = 0.0;
    if (useStratification)
    {
        binnedCandidates.resize(params_.lc_distance_bins);
        binWidth = (maxDist - minDist) / static_cast<double>(params_.lc_distance_bins);
    }

    // Single vector for non-stratified approaches
    std::vector<LoopCandidate> candidates;

    // ========================================================================
    // STEP 1: Generate and score all candidates
    // ========================================================================

    for (size_t i = 0; i < sm.size(); i++)
    {
        const auto pose_i = state_.get_pose(i);

        for (size_t j = i + params_.min_frames_between_lc; j < sm.size(); j++)
        {
            // Check if already evaluated
            const auto frameGroup_i = mrpt::round(static_cast<double>(i) / frameGroup);
            const auto frameGroup_j = mrpt::round(static_cast<double>(j) / frameGroup);

            const auto IDs = std::make_pair(
                std::min<frame_id_t>(frameGroup_i, frameGroup_j),
                std::max<frame_id_t>(frameGroup_i, frameGroup_j));

            if (alreadyChecked.count(IDs) != 0)
            {
                continue;
            }

            // Compute spatial distance
            const auto   pose_j   = state_.get_pose(j);
            const double distance = (pose_i.translation() - pose_j.translation()).norm();

            // Apply distance constraints
            if (distance < minDist || distance > maxDist)
            {
                continue;
            }

            // Verify valid observations
            const auto& [_, sf_i, __]     = sm.get(i);  // NOLINT(bugprone-reserved-identifier)
            const auto& [___, sf_j, ____] = sm.get(j);  // NOLINT(bugprone-reserved-identifier)

            if (!frame_has_mapping_observations(*sf_i) || !frame_has_mapping_observations(*sf_j))
            {
                continue;
            }

            // Create candidate
            LoopCandidate lc;
            lc.frame_i  = i;
            lc.frame_j  = j;
            lc.distance = distance;

            // Compute score based on selected strategy
            switch (params_.lc_candidate_strategy)
            {
                case Parameters::CandidateSelectionStrategy::PROXIMITY_ONLY:
                    lc.score = score_proximity_only(distance);
                    break;

                case Parameters::CandidateSelectionStrategy::DISTANCE_STRATIFIED:
                    lc.score = score_stratified(distance, minDist, maxDist, i, j, sm.size());
                    break;

                case Parameters::CandidateSelectionStrategy::MULTI_OBJECTIVE:
                    lc.score = score_multi_objective(
                        distance, minDist, maxDist, i, j, sm.size(), selectedDistances,
                        params_.lc_weight_proximity, params_.lc_weight_frame_separation,
                        params_.lc_weight_diversity, params_.lc_weight_coverage);
                    break;
            }

            // Add to appropriate container
            if (useStratification)
            {
                // Determine bin index
                const size_t binIdx = std::min(
                    static_cast<size_t>((distance - minDist) / binWidth),
                    params_.lc_distance_bins - 1);
                binnedCandidates[binIdx].push_back(lc);
            }
            else
            {
                candidates.push_back(lc);
            }

            if (PRINT_LC_SCORES)
            {
                MRPT_LOG_DEBUG_STREAM(
                    "Candidate: " << i << " <-> " << j << " dist=" << distance
                                  << " score=" << lc.score);
            }
        }
    }

    // ========================================================================
    // STEP 2: Select final candidates based on strategy
    // ========================================================================

    std::vector<LoopCandidate> finalCandidates;

    if (useStratification)
    {
        // Strategy: Sample proportionally from each distance bin

        const size_t baseCandidatesPerBin = params_.max_lc_candidates / params_.lc_distance_bins;
        const size_t extraCandidates      = params_.max_lc_candidates % params_.lc_distance_bins;

        for (size_t binIdx = 0; binIdx < params_.lc_distance_bins; binIdx++)
        {
            auto& bin = binnedCandidates[binIdx];

            if (bin.empty())
            {
                continue;
            }

            // Sort within bin
            std::sort(
                bin.begin(), bin.end(),
                [](const LoopCandidate& a, const LoopCandidate& b) { return a.score > b.score; });

            // Determine number to select from this bin
            size_t toTake = baseCandidatesPerBin;
            if (binIdx < extraCandidates)
            {
                toTake++;
            }
            toTake = std::min(toTake, bin.size());

            // Add top candidates from bin
            for (size_t k = 0; k < toTake; k++)
            {
                finalCandidates.push_back(bin[k]);
            }

            if (params_.lc_verbose_candidate_selection)
            {
                const double binMin = minDist + static_cast<double>(binIdx) * binWidth;
                const double binMax = minDist + static_cast<double>(binIdx + 1) * binWidth;
                MRPT_LOG_INFO_STREAM(
                    "Bin [" << binMin << ", " << binMax << "] m: " << bin.size()
                            << " candidates, selected " << toTake);
            }
        }

        // Final global sort
        std::sort(
            finalCandidates.begin(), finalCandidates.end(),
            [](const LoopCandidate& a, const LoopCandidate& b) { return a.score > b.score; });
    }
    else
    {
        // Strategy: Simple top-K selection by score

        std::sort(
            candidates.begin(), candidates.end(),
            [](const LoopCandidate& a, const LoopCandidate& b) { return a.score > b.score; });

        finalCandidates = std::move(candidates);
    }

    // Limit to max candidates
    if (finalCandidates.size() > params_.max_lc_candidates)
    {
        finalCandidates.resize(params_.max_lc_candidates);
    }

    // ========================================================================
    // STEP 3: Log statistics (if verbose or always at INFO level)
    // ========================================================================

    if (!finalCandidates.empty() && (params_.lc_verbose_candidate_selection || PRINT_LC_SCORES))
    {
        std::vector<double> distances;
        distances.reserve(finalCandidates.size());
        for (const auto& lc : finalCandidates)
        {
            distances.push_back(lc.distance);
        }

        const double minSelectedDist = *std::min_element(distances.begin(), distances.end());
        const double maxSelectedDist = *std::max_element(distances.begin(), distances.end());
        const double sumDist         = std::accumulate(distances.begin(), distances.end(), 0.0);
        const double meanDist        = sumDist / static_cast<double>(distances.size());

        // Compute standard deviation
        double variance = 0.0;
        for (const auto d : distances)
        {
            const double diff = d - meanDist;
            variance += diff * diff;
        }
        const double stdDist = std::sqrt(variance / static_cast<double>(distances.size()));

        // Compute coefficient of variation (normalized measure of variance)
        const double cv = (meanDist > 0.0) ? (stdDist / meanDist) : 0.0;

        MRPT_LOG_INFO_STREAM(
            "Selected " << finalCandidates.size() << " LC candidates. "
                        << "Distance: [" << minSelectedDist << ", " << maxSelectedDist << "] m, "
                        << "mean=" << meanDist << " m, "
                        << "std=" << stdDist << " m, "
                        << "CV=" << cv);
    }

    return finalCandidates;
}

bool FrameToFrameLoopClosure::process_loop_candidate(const LoopCandidate& lc)
{
    using gtsam::symbol_shorthand::X;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "process_loop_candidate");

    const size_t threadIdx = 0;  // Use first thread for now

    // Generate point clouds for both frames
    auto pc_i = generate_frame_pointcloud(lc.frame_i, threadIdx);
    auto pc_j = generate_frame_pointcloud(lc.frame_j, threadIdx);

    if (!pc_i || !pc_j)
    {
        MRPT_LOG_WARN_STREAM(
            "Failed to generate point clouds for LC " << lc.frame_i << " <-> " << lc.frame_j);
        return false;
    }

    // Initial guess from current graph
    const auto pose_i    = state_.get_pose(lc.frame_i);
    const auto pose_j    = state_.get_pose(lc.frame_j);
    const auto initGuess = (pose_j - pose_i).asTPose();

    // Run ICP
    auto& pts = state_.perThreadState_.at(threadIdx);

    update_dynamic_variables(lc.frame_j, threadIdx);

    mp2p_icp::Results icp_result;
    pts.icp->align(*pc_j, *pc_i, initGuess, params_.icp_parameters, icp_result);

    const auto poseDelta = (icp_result.optimal_tf.getMeanVal().asTPose() - initGuess);

    MRPT_LOG_INFO_STREAM(
        "ICP " << lc.frame_i << " <-> " << lc.frame_j << " distance=" << lc.distance
               << " score=" << lc.score << " icp_quality=" << (100.0 * icp_result.quality)
               << "% iters=" << icp_result.nIterations << " Δp=" << poseDelta.translation().norm()
               << " [m] ΔR="
               << mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(poseDelta.getRotationMatrix()).norm())
               << " [deg]");

    if (icp_result.quality < params_.min_icp_goodness)
    {
        return false;
    }

    // Add ICP edge to graph
    const auto deltaPose = mrpt::gtsam_wrappers::toPose3(icp_result.optimal_tf.mean);

    gtsam::Vector6 sigmas;
    const auto     covDiag = icp_result.optimal_tf.cov.asEigen().diagonal().array().sqrt();

    sigmas << covDiag[5] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[4] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[3] + mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang),
        covDiag[0] + params_.icp_edge_additional_noise_xyz,
        covDiag[1] + params_.icp_edge_additional_noise_xyz,
        covDiag[2] + params_.icp_edge_additional_noise_xyz;

    auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    // Robust kernel:
    auto robustNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::GemanMcClure::Create(params_.icp_edge_robust_param),
        edgeNoise);

    state_.graphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(lc.frame_i), X(lc.frame_j), deltaPose, robustNoise);

    accepted_lc_edges_.emplace_back(lc.frame_i, lc.frame_j);

    return true;
}

MRPT_TODO("Use mp2p_icp::update_velocity_buffer_from_obs() once mp2p_icp 2.5.0 is available");
namespace
{
void processLocalVelocityBuffer(
    const mrpt::obs::CObservation::Ptr& obs, mp2p_icp::ParameterSource& ps)
{
    auto obsComment = std::dynamic_pointer_cast<mrpt::obs::CObservationComment>(obs);
    if (!obsComment)
    {
        return;
    }

    const auto commentYaml = [&]()
    {
        try
        {
            return mrpt::containers::yaml::FromText(obsComment->text);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing YAML in comment: " << e.what() << "\n";
            return mrpt::containers::yaml();
        }
    }();

    if (!commentYaml.isMap() || !commentYaml.has("local_velocity_buffer"))
    {
        return;
    }

    const auto lvb = commentYaml["local_velocity_buffer"];
    if (!lvb.isMap())
    {
        std::cerr << "Error: 'local_velocity_buffer' field is not a map!\n";
        return;
    }

    try
    {
        ps.localVelocityBuffer.fromYAML(lvb);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error parsing 'local_velocity_buffer': " << e.what() << "\n";
        return;
    }
};
}  // namespace

mp2p_icp::metric_map_t::Ptr FrameToFrameLoopClosure::generate_frame_pointcloud(
    frame_id_t frameId, size_t threadIdx)
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "generate_frame_pointcloud");

    ASSERT_(state_.sm);
    const auto& [pose, sf, twist] = state_.sm->get(frameId);

    if (!frame_has_mapping_observations(*sf))
    {
        return {};
    }

    auto& pts         = state_.perThreadState_.at(threadIdx);
    auto  observation = mp2p_icp::metric_map_t::Create();

    // First, search for velocity buffer data:
    for (const auto& obs : *sf)
    {
        ASSERT_(obs);
        processLocalVelocityBuffer(obs, pts.parameter_source);
    }

    update_dynamic_variables(frameId, threadIdx);

    // Next, do the actual sensor data processing:

    try
    {
        // Generate point cloud from observations
        for (const auto& obs : *sf)
        {
            mp2p_icp_filters::apply_generators(pts.obs_generators, *obs, *observation);
        }
    }
    catch (const std::exception& e)
    {
        // If the exception msg contains "Assert file existence failed", it's due to missing
        // external files. Emit a warning and return an empty cloud for this frame,
        // but continue with the rest without quitting.
        const std::string errMsg = e.what();
        if (errMsg.find("Assert file existence failed") != std::string::npos)
        {
            MRPT_LOG_WARN_STREAM(
                "Frame " << frameId << ": Skipping observation due to missing external files: "
                         << first_n_lines(errMsg, 3));
            return {};
        }
        throw;  // Rethrow other exceptions
    }

    // Apply filters
    mp2p_icp_filters::apply_filter_pipeline(pts.pc_filter, *observation, profiler_);

    return observation;
}

double FrameToFrameLoopClosure::optimize_graph()
{
    mrpt::system::CTimeLoggerEntry tle(profiler_, "optimize_graph");

    auto lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();

    ASSERT_(!state_.graphFG.empty());

    const auto N_1 = 1.0 / static_cast<double>(state_.graphFG.size());

    const double errInit1  = state_.graphFG.error(state_.graphValues);
    const double rmseInit1 = std::sqrt(errInit1 * N_1);

    gtsam::LevenbergMarquardtOptimizer lm1(state_.graphFG, state_.graphValues, lmParams);
    const auto                         optimalValues = lm1.optimize();

    const double errEnd1  = state_.graphFG.error(optimalValues);
    const double rmseEnd1 = std::sqrt(errEnd1 * N_1);

    // Compute largest pose change
    double largestDelta = 0.0;
    using gtsam::symbol_shorthand::X;

    for (size_t i = 0; i < state_.sm->size(); i++)
    {
        const auto newPose = mrpt::gtsam_wrappers::toTPose3D(optimalValues.at<gtsam::Pose3>(X(i)));
        const auto oldPose =
            mrpt::gtsam_wrappers::toTPose3D(state_.graphValues.at<gtsam::Pose3>(X(i)));

        const double delta = mrpt::poses::CPose3D(oldPose - newPose).translation().norm();
        mrpt::keep_max(largestDelta, delta);
    }

    // Save new optimal values
    state_.graphValues = optimalValues;

    // Compute marginals:
    state_.graphMarginals.emplace(state_.graphFG, state_.graphValues);

    // Logging:
    auto bckCol =
        mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO);
    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) =
        mrpt::system::ConsoleForegroundColor::BRIGHT_GREEN;

    MRPT_LOG_INFO_STREAM(
        "Graph optimized: " << lm1.iterations() << " iters, RMSE " << rmseInit1 << " -> "
                            << rmseEnd1 << ", largest delta: " << largestDelta << " m");

    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) = bckCol;

    return largestDelta;
}

mrpt::poses::CPose3D FrameToFrameLoopClosure::frame_pose_in_simplemap(frame_id_t frameId) const
{
    ASSERT_(state_.sm);
    const auto& [pose, sf, twist] = state_.sm->get(frameId);
    ASSERT_(pose);
    return pose->getMeanVal();
}

mrpt::poses::CPose3D FrameToFrameLoopClosure::State::get_pose(frame_id_t id) const
{
    using gtsam::symbol_shorthand::X;
    return mrpt::poses::CPose3D(
        mrpt::gtsam_wrappers::toTPose3D(graphValues.at<gtsam::Pose3>(X(id))));
}

mrpt::math::CMatrixDouble66 FrameToFrameLoopClosure::State::get_pose_cov(frame_id_t id) const
{
    using gtsam::symbol_shorthand::X;
    ASSERT_(graphMarginals.has_value());

    return mrpt::gtsam_wrappers::to_mrpt_se3_cov6(graphMarginals->marginalCovariance(X(id)));
}

void FrameToFrameLoopClosure::update_dynamic_variables(frame_id_t frameId, size_t threadIdx)
{
    auto& pts = state_.perThreadState_.at(threadIdx);
    auto& ps  = pts.parameter_source;

    const auto& [pose, sf, twist] = state_.sm->get(frameId);

    // Set twist for deskewing
    mrpt::math::TTwist3D twistForIcp = {0, 0, 0, 0, 0, 0};
    if (twist)
    {
        twistForIcp = *twist;
    }

    ps.updateVariable("vx", twistForIcp.vx);
    ps.updateVariable("vy", twistForIcp.vy);
    ps.updateVariable("vz", twistForIcp.vz);
    ps.updateVariable("wx", twistForIcp.wx);
    ps.updateVariable("wy", twistForIcp.wy);
    ps.updateVariable("wz", twistForIcp.wz);

    if (!pts.expr_threshold_sigma_final.is_compiled())
    {
        pts.expr_threshold_sigma_final.compile(
            params_.threshold_sigma_final, {}, "expr_threshold_sigma_final");

        pts.expr_threshold_sigma_initial.compile(
            params_.threshold_sigma_initial, {}, "expr_threshold_sigma_initial");
    }

    ps.updateVariable("SIGMA_INIT", pts.expr_threshold_sigma_initial.eval());
    ps.updateVariable("SIGMA_FINAL", pts.expr_threshold_sigma_final.eval());
    ps.updateVariable("ESTIMATED_SENSOR_MAX_RANGE", params_.max_sensor_range);

    // This will be overwritten by the actual ICP loop later on,
    // but we need to define all variables before building a local map:
    ps.updateVariable("ICP_ITERATION", 0);

    ps.realize();
}

void FrameToFrameLoopClosure::save_3d_scene_initial_files() const
{
    ASSERT_(state_.sm);
    const auto& sm     = *state_.sm;
    const auto& prefix = params_.debug_files_prefix;

    const auto pathColor = mrpt::img::TColorf(
                               params_.scene_path_color_r, params_.scene_path_color_g,
                               params_.scene_path_color_b, params_.scene_path_color_a)
                               .asTColor();

    // 1) Initial path edges
    {
        auto lines = mrpt::opengl::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_path_line_width);
        lines->setColor_u8(pathColor);

        for (size_t i = 1; i < sm.size(); i++)
        {
            const auto p0 = frame_pose_in_simplemap(i - 1).translation();
            const auto p1 = frame_pose_in_simplemap(i).translation();
            lines->appendLine(p0, p1);
        }

        mrpt::opengl::Scene scene;
        scene.insert(lines);
        const auto fn = prefix + "initial_path_edges.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }

    // 2) Initial keyframe points
    {
        auto pts = mrpt::opengl::CPointCloud::Create();
        pts->setPointSize(params_.scene_keyframe_point_size);
        pts->setColor_u8(pathColor);

        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto p = frame_pose_in_simplemap(i).translation();
            pts->insertPoint(p);
        }

        mrpt::opengl::Scene scene;
        scene.insert(pts);
        const auto fn = prefix + "initial_keyframe_points.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }
}

void FrameToFrameLoopClosure::save_3d_scene_files() const
{
    ASSERT_(state_.sm);
    const auto& sm     = *state_.sm;
    const auto& prefix = params_.debug_files_prefix;

    // 1) Path edges: lines connecting consecutive keyframes
    {
        auto lines = mrpt::opengl::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_path_line_width);
        lines->setColor_u8(mrpt::img::TColorf(
                               params_.scene_path_color_r, params_.scene_path_color_g,
                               params_.scene_path_color_b, params_.scene_path_color_a * 255)
                               .asTColor());

        for (size_t i = 1; i < sm.size(); i++)
        {
            const auto p0 = state_.get_pose(i - 1).translation();
            const auto p1 = state_.get_pose(i).translation();
            lines->appendLine(p0, p1);
        }

        mrpt::opengl::Scene scene;
        scene.insert(lines);
        const auto fn = prefix + "path_edges.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }

    // 2) Keyframe points
    {
        auto pts = mrpt::opengl::CPointCloud::Create();
        pts->setPointSize(params_.scene_keyframe_point_size);
        pts->setColor_u8(mrpt::img::TColorf(
                             params_.scene_path_color_r, params_.scene_path_color_g,
                             params_.scene_path_color_b, params_.scene_path_color_a)
                             .asTColor());

        for (size_t i = 0; i < sm.size(); i++)
        {
            const auto p = state_.get_pose(i).translation();
            pts->insertPoint(p);
        }

        mrpt::opengl::Scene scene;
        scene.insert(pts);
        const auto fn = prefix + "keyframe_points.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }

    // 3) Loop closure edges
    {
        auto lines = mrpt::opengl::CSetOfLines::Create();
        lines->setLineWidth(params_.scene_lc_line_width);
        lines->setColor_u8(mrpt::img::TColorf(
                               params_.scene_lc_color_r, params_.scene_lc_color_g,
                               params_.scene_lc_color_b, params_.scene_lc_color_a)
                               .asTColor());

        for (const auto& [fi, fj] : accepted_lc_edges_)
        {
            const auto p0 = state_.get_pose(fi).translation();
            const auto p1 = state_.get_pose(fj).translation();
            lines->appendLine(p0, p1);
        }

        mrpt::opengl::Scene scene;
        scene.insert(lines);
        const auto fn = prefix + "lc_edges.3Dscene";
        if (scene.saveToFile(fn))
        {
            MRPT_LOG_INFO_STREAM("Saved 3D scene: " << fn);
        }
        else
        {
            MRPT_LOG_WARN_STREAM("Failed to save 3D scene: " << fn);
        }
    }
}

void FrameToFrameLoopClosure::save_trajectory_as_tum(
    const std::string& filename, bool saveCovariancesToo) const
{
    ASSERT_(state_.sm);

    mrpt::poses::CPose3DInterpolator path;
    mrpt::math::CMatrixDouble        pathSigmas;

    if (saveCovariancesToo)
    {
        ASSERT_(state_.graphMarginals.has_value());
        pathSigmas.setZero(state_.sm->size(), 6);
    }

    for (size_t id = 0; id < state_.sm->size(); id++)
    {
        const auto& [oldPose, sf, twist] = state_.sm->get(id);
        const auto newPose               = state_.get_pose(id);

        if (sf->empty())
        {
            MRPT_LOG_WARN_STREAM("Frame " << id << " has no observations, skipping in trajectory");
            continue;
        }
        const auto t = sf->getObservationByIndex(0)->timestamp;

        path.insert(t, newPose);

        if (saveCovariancesToo)
        {
            const auto& cov = state_.get_pose_cov(id);

            for (int i = 0; i < 6; i++)
            {
                pathSigmas(id, i) = std::sqrt(cov(i, i));
            }
        }
    }

    path.saveToTextFile_TUM(filename);

    if (saveCovariancesToo)
    {
        const std::string header =
            "# sigma_x_m sigma_y_m sigma_z_m sigma_yaw_rad sigma_pitch_rad sigma_roll_rad";
        pathSigmas.saveToTextFile(
            mrpt::system::fileNameChangeExtension(filename, "cov"), mrpt::math::MATRIX_FORMAT_ENG,
            false, header);
    }

    MRPT_LOG_INFO_STREAM("Saved trajectory to: " << filename);
}