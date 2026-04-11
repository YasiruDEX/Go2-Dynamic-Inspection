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

// MRPT:
#include <mola_georeferencing/simplemap_georeference.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CEllipsoid2D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>

// MOLA:
#include <mola_relocalization/relocalization.h>
#include <mola_sm_loop_closure/SimplemapLoopClosure.h>
#include <mola_yaml/yaml_helpers.h>

// MRPT graphs:
#include <mrpt/graphs/dijkstra.h>

// visualization:
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/graph_tools.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
// #include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

using namespace mola;

IMPLEMENTS_SERIALIZABLE(SimplemapLoopClosure, LoopClosureInterface, mola)

namespace
{
const bool PRINT_ALL_SCORES = mrpt::get_env<bool>("PRINT_ALL_SCORES", false);
const bool SAVE_LCS         = mrpt::get_env<bool>("SAVE_LCS", false);
const bool SAVE_TREES       = mrpt::get_env<bool>("SAVE_TREES", false);
const bool PRINT_FG_ERRORS  = mrpt::get_env<bool>("PRINT_FG_ERRORS", false);

const bool ADD_GNSS_FACTORS_2ND_STAGE = mrpt::get_env<bool>("ADD_GNSS_FACTORS_2ND_STAGE", true);

const bool DEBUG_PRINT_BETWEEN_EDGES = mrpt::get_env<bool>("DEBUG_PRINT_BETWEEN_EDGES", false);

mrpt::math::TBoundingBox SimpleMapBoundingBox(const mrpt::maps::CSimpleMap& sm)
{
    // estimate path bounding box:
    auto bbox = mrpt::math::TBoundingBox::PlusMinusInfinity();

    for (const auto& [pose, sf, twist] : sm)
    {
        const auto p = pose->getMeanVal().asTPose();
        bbox.updateWithPoint(p.translation());
    }

    return bbox;
}

// TODO: Remove these helpers once all ROS distros have mola_georefrencing >= 2.1.0
//
// Helper to check if something is an optional (false for plain types)
template <typename T>
constexpr bool is_nullopt(const T&)  // NOLINT
{
    return false;
}

template <typename T>
constexpr bool is_nullopt(const std::optional<T>& o)
{
    return !o.has_value();
}

// Unified access: works for both Foo and std::optional<Foo>
template <typename T>
constexpr auto& deref(T& x)
{
    return x;
}

template <typename T>
constexpr auto& deref(std::optional<T>& x)
{
    return *x;
}

}  // namespace

SimplemapLoopClosure::SimplemapLoopClosure()
{
    mrpt::system::COutputLogger::setLoggerName("SimplemapLoopClosure");
    threads_.name("sm_localmaps_build");
}

void SimplemapLoopClosure::initialize(const mrpt::containers::yaml& c)
{
    MRPT_TRY_START

    // Load params:
    const auto cfg = c["params"];

    YAML_LOAD_REQ(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, profiler_enabled, bool);
    YAML_LOAD_REQ(params_, submap_max_length_wrt_map, double);
    YAML_LOAD_REQ(params_, submap_max_absolute_length, double);
    YAML_LOAD_REQ(params_, submap_min_absolute_length, double);
    YAML_LOAD_REQ(params_, max_time_between_kfs_to_break_submap, double);
    YAML_LOAD_OPT(params_, do_first_gross_relocalize, bool);
    YAML_LOAD_OPT(params_, do_montecarlo_icp, bool);
    YAML_LOAD_OPT(params_, assume_planar_world, bool);
    YAML_LOAD_OPT(params_, use_gnss, bool);
    YAML_LOAD_OPT(params_, gnss_minimum_uncertainty_xyz, double);

    YAML_LOAD_REQ(params_, threshold_sigma_initial, std::string);
    YAML_LOAD_REQ(params_, threshold_sigma_final, std::string);
    YAML_LOAD_REQ(params_, max_sensor_range, double);

    YAML_LOAD_REQ(params_, icp_edge_robust_param, double);
    YAML_LOAD_REQ(params_, icp_edge_worst_multiplier, double);

    YAML_LOAD_REQ(params_, icp_edge_additional_noise_xyz, double);
    YAML_LOAD_REQ(params_, icp_edge_additional_noise_ang_deg, double);
    YAML_LOAD_REQ(params_, input_odometry_edge_additional_noise_xyz, double);
    YAML_LOAD_REQ(params_, input_odometry_edge_additional_noise_ang_deg, double);

    YAML_LOAD_OPT(params_, input_edges_uncertainty_multiplier, double);
    YAML_LOAD_OPT(params_, max_number_lc_candidates, uint32_t);
    YAML_LOAD_OPT(params_, max_number_lc_candidates_per_submap, uint32_t);
    YAML_LOAD_OPT(params_, min_lc_uncertainty_ratio_to_draw_several_samples, double);

    YAML_LOAD_OPT(params_, largest_delta_for_reconsider_all, double);

    YAML_LOAD_OPT(params_, min_volume_intersection_ratio_for_lc_candidate, double);

    YAML_LOAD_OPT(params_, save_submaps_viz_files, bool);

    // system-wide profiler:
    profiler_.enable(params_.profiler_enabled);

    ENSURE_YAML_ENTRY_EXISTS(c, "icp_settings");
    ASSERT_(c["insert_observation_into_local_map"].isSequence());

    mrpt::system::CTimeLoggerEntry tlePcInit(profiler_, "filterPointCloud_initialize");

    for (size_t threadIdx = 0; threadIdx < state_.perThreadState_.size(); threadIdx++)
    {
        auto& pts = state_.perThreadState_.at(threadIdx);

        const auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(c["icp_settings"]);

        pts.icp                = icp;
        params_.icp_parameters = icpParams;

        // Attach all ICP instances to the parameter source for dynamic
        // parameters:
        pts.icp->attachToParameterSource(pts.parameter_source);

        // Obs2map merge pipeline:
        pts.obs2map_merge =
            mp2p_icp_filters::filter_pipeline_from_yaml(c["insert_observation_into_local_map"]);

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(pts.obs2map_merge, pts.parameter_source);

        ASSERT_(!pts.obs2map_merge.empty());

        // Create lidar segmentation algorithm:
        // Observation -> map generator:
        if (c.has("observations_generator") && !c["observations_generator"].isNullNode())
        {
            pts.obs_generators =
                mp2p_icp_filters::generators_from_yaml(c["observations_generator"]);
        }
        else
        {
            std::cout << "[warning] Using default mp2p_icp_filters::Generator for "
                         "observations since no YAML 'observations_generator' entry "
                         "was given\n";

            auto defaultGen = mp2p_icp_filters::Generator::Create();
            defaultGen->initialize({});
            pts.obs_generators.push_back(defaultGen);
        }

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(pts.obs_generators, pts.parameter_source);

        if (c.has("observations_filter"))
        {
            pts.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(c["observations_filter"]);

            // Attach to the parameter source for dynamic parameters:
            mp2p_icp::AttachToParameterSource(pts.pc_filter, pts.parameter_source);
        }

        // Local map generator:
        if (c.has("localmap_generator") && !c["localmap_generator"].isNullNode())
        {
            pts.local_map_generators =
                mp2p_icp_filters::generators_from_yaml(c["localmap_generator"]);
        }
        else
        {
            ASSERT_(
                "Providing a 'localmap_generator' is mandatory in this "
                "application.");
        }
        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(pts.local_map_generators, pts.parameter_source);

        // submaps final stage filter:
        pts.submap_final_filter =
            mp2p_icp_filters::filter_pipeline_from_yaml(c["submap_final_filter"]);

        // Attach to the parameter source for dynamic parameters:
        mp2p_icp::AttachToParameterSource(pts.submap_final_filter, pts.parameter_source);

    }  // end for threadIdx
    tlePcInit.stop();

    state_.initialized = true;

    MRPT_TRY_END
}

namespace
{
bool sf_has_real_mapping_observations(const mrpt::obs::CSensoryFrame& sf)
{
    if (sf.empty())
    {
        return false;
    }
    if (auto oPC = sf.getObservationByClass<mrpt::obs::CObservationPointCloud>(); oPC)
    {
        return true;
    }

    if (auto o2D = sf.getObservationByClass<mrpt::obs::CObservation2DRangeScan>(); o2D)
    {
        return true;
    }

    if (auto o3D = sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>(); o3D)
    {
        return true;
    }

    if (auto oVl = sf.getObservationByClass<mrpt::obs::CObservationVelodyneScan>(); oVl)
    {
        return true;
    }

    // We don't recognize any valid mapping-suitable observation in the SF.
    return false;
}

std::optional<mrpt::Clock::time_point> sf_timestamp(const mrpt::obs::CSensoryFrame& sf)
{
    for (const auto& o : sf)
    {
        if (!o)
        {
            continue;
        }
        if (o->timestamp == mrpt::Clock::time_point())
        {
            continue;
        }
        return o->timestamp;
    }
    return {};
}

}  // namespace

// Find and apply loop closures in the input/output simplemap
void SimplemapLoopClosure::process(mrpt::maps::CSimpleMap& sm)
{
    using namespace std::string_literals;

    ASSERT_(state_.initialized);

    state_.sm = &sm;

    // Minimum to have a large-enough topological loop closure:
    // ASSERT_GT_(sm.size(), 3 * params_.submap_keyframe_count);

    // Build submaps:
    // ------------------------------------------------
    const auto detectedSubMaps = detect_sub_maps();

    // process pending submap creation, in parallel threads:
    const size_t nSubMaps = detectedSubMaps.size();

    for (submap_id_t submapId = 0; submapId < nSubMaps; submapId++)
    {
        // Modify the submaps[] std::map here in this main thread:
        SubMap& submap = state_.submaps[submapId];
        submap.id      = submapId;

        build_submap_from_kfs_into(detectedSubMaps.at(submapId), submap);

        MRPT_LOG_INFO_STREAM("Done with submap #" << submapId << " / " << nSubMaps);
    }

    // Build a graph with the submaps:
    // -----------------------------------------------
    // Nodes:
    for (const auto& [id, submap] : state_.submaps)
    {
        // These are poses without uncertainty:
        auto& globalPose = state_.submapsGraph.nodes[id];
        globalPose       = submap.global_pose;
    }
    // Edges: one between adjacent submaps in traverse order:
    {
        std::optional<const SubMap*> lastSubmap;
        for (const auto& [id, submap] : state_.submaps)
        {
            if (lastSubmap)
            {
                // add edge: i-1 => i
                const auto last_id = lastSubmap.value()->id;
                const auto this_id = id;

                const auto& thisGlobalPose = state_.submapsGraph.nodes[this_id];
                const auto& lastGlobalPose = state_.submapsGraph.nodes[last_id];

                mrpt::poses::CPose3DPDFGaussian relPose;
                relPose.mean = thisGlobalPose - lastGlobalPose;

                // Note: this uncertainty for the SUBMAPS graph is used to
                // estimate submap overlap probabilities:
                using namespace mrpt::literals;  // _deg

                relPose.cov.setDiagonal(
                    {mrpt::square(0.10), mrpt::square(0.10), mrpt::square(0.15),
                     mrpt::square(1.0_deg), mrpt::square(1.5_deg), mrpt::square(1.0_deg)});

                state_.submapsGraph.insertEdge(last_id, this_id, relPose);
            }

            lastSubmap = &submap;
        }
    }

    // Build a graph with the low-level keyframes:
    // -----------------------------------------------
    using gtsam::symbol_shorthand::X;  // poses SE(3)

    gtsam::NonlinearFactor::shared_ptr x0prior;

    for (const auto& [submapId, submap] : state_.submaps)
    {
        for (const auto id : submap.kf_ids)
        {
            const auto         pose_i = keyframe_pose_in_simplemap(id);
            const gtsam::Pose3 p      = mrpt::gtsam_wrappers::toPose3(pose_i);

            state_.kfGraphValues.insert(X(id), p);

            // anchor for first KF: only if we don't have GNSS
            if (!x0prior)
            {
#if GTSAM_USES_BOOST
                x0prior = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(id), p);
#else
                x0prior = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(id), p);
#endif
            }
        }
    }

    // anchor for X(0):
    state_.kfGraphFG.push_back(x0prior);
    state_.kfGraphFGRobust.push_back(x0prior);

    // create edges: i -> i-1
    for (size_t i = 1; i < sm.size(); i++)
    {
        // Extract cov from simplemap cov from icp odometry:
        mrpt::poses::CPose3DPDFGaussian ppi;
        mrpt::poses::CPose3DPDFGaussian ppim1;
        {
            const auto& [pose_i, sf_i, twist_i]       = state_.sm->get(i);
            const auto& [pose_im1, sf_im1, twist_im1] = state_.sm->get(i - 1);

            ppi.copyFrom(*pose_i);
            ppim1.copyFrom(*pose_im1);
        }

        const mrpt::poses::CPose3DPDFGaussian relPose = ppi - ppim1;

        gtsam::Vector6 sigmasXYZYPR = relPose.cov.asEigen().diagonal().array().sqrt().eval();

        // Enlarge uncertainty?
        sigmasXYZYPR *= params_.input_edges_uncertainty_multiplier;

        // Minimum uncertainty?
        for (int k = 0; k < 3; k++)
        {
            sigmasXYZYPR[k] += params_.input_odometry_edge_additional_noise_xyz;
            sigmasXYZYPR[3 + k] +=
                mrpt::DEG2RAD(params_.input_odometry_edge_additional_noise_ang_deg);
        }

        if (DEBUG_PRINT_BETWEEN_EDGES)
        {
            MRPT_LOG_INFO_STREAM(
                "[FG] Adding edge: " << i - 1 << " => " << i << " pose: " << relPose.getPoseMean()
                                     << " sigmas: " << sigmasXYZYPR.transpose() << "\n"
                                     << "relPose: " << relPose.cov << "\n"
                                     << "ppi: " << ppi << "\n"
                                     << "ppim1: " << ppim1 << "\n\n");
        }

        const gtsam::Pose3 deltaPose = mrpt::gtsam_wrappers::toPose3(relPose.getPoseMean());

        gtsam::Vector6 sigmas;
        sigmas << sigmasXYZYPR[5], sigmasXYZYPR[4], sigmasXYZYPR[3],  //
            sigmasXYZYPR[0], sigmasXYZYPR[1], sigmasXYZYPR[2];

        auto edgeNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

#if GTSAM_USES_BOOST
        auto f = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);
#else
        auto f = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i - 1), X(i), deltaPose, edgeNoise);
#endif

        state_.kfGraphFG += f;
        state_.kfGraphFGRobust += f;
    }

    // GNSS Edges: additional edges in both graphs:
    if (state_.globalGeoRef.has_value())
    {
        const auto  ref_id    = state_.globalGeoRefSubmapId;
        const auto& refSubmap = state_.submaps.at(ref_id);

        for (const auto& [id, submap] : state_.submaps)
        {
            // has this submap GNSS?
            if (!submap.geo_ref)
            {
                continue;
            }

            // add edge: gpsRefId => id
            const auto this_id = id;

            // T_0_i = (T_enu_0)⁻¹ · T_enu_i (notes picture! pass to paper)
            const auto T_enu_0 = refSubmap.geo_ref->T_enu_to_map;
            auto       T_0_enu = -T_enu_0;
            T_0_enu.cov.setZero();  // Ignore uncertainty of this first T

            const auto T_enu_i = submap.geo_ref->T_enu_to_map;

            mrpt::poses::CPose3DPDFGaussian relPose /*T_0_i*/ = T_0_enu + T_enu_i;

            // 1) Add edge to submaps-level graph:
            state_.submapsGraph.insertEdge(ref_id, this_id, relPose);

            // 2) Add edge to low-level keyframe graph:
            const gtsam::Pose3 deltaPose = mrpt::gtsam_wrappers::toPose3(relPose.getPoseMean());

            auto edgeNoise = gtsam::noiseModel::Gaussian::Covariance(
                mrpt::gtsam_wrappers::to_gtsam_se3_cov6_reordering(relPose.cov));

#if 0
            const double gnss_edge_robust_param = 3.0;

            gtsam::noiseModel::Base::shared_ptr edgeRobNoise =
                gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                        gnss_edge_robust_param),
                    edgeNoise);
#else
            auto edgeRobNoise = edgeNoise;
#endif

            const auto refKfId = *refSubmap.kf_ids.begin();
            const auto curKfId = *submap.kf_ids.begin();

            MRPT_LOG_DEBUG_STREAM(
                "GNSS edge #" << refKfId << " => #" << curKfId << " relPose: " << relPose
                              << "\n gtsam:" << deltaPose << "\n cov:\n"
                              << mrpt::gtsam_wrappers::to_gtsam_se3_cov6_reordering(relPose.cov));

            {
                const auto p0 = state_.kfGraphValues.at<gtsam::Pose3>(X(refKfId));
                const auto pi = state_.kfGraphValues.at<gtsam::Pose3>(X(curKfId));
                const auto p01pre =
                    mrpt::gtsam_wrappers::toTPose3D(pi) - mrpt::gtsam_wrappers::toTPose3D(p0);

                MRPT_LOG_DEBUG_STREAM(
                    "FG GNSS edge:\n"
                    "p01: "
                    << p01pre
                    << "\n"
                       "new: "
                    << relPose.getPoseMean().asTPose());
            }

            state_.kfGraphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(refKfId), X(curKfId), deltaPose, edgeNoise);

            if (ADD_GNSS_FACTORS_2ND_STAGE)
            {
                state_.kfGraphFGRobust.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    X(refKfId), X(curKfId), deltaPose, edgeRobNoise);
            }
        }

        if (params_.save_submaps_viz_files)
        {  // Save viz of initial state:
            VizOptions opts;
            auto       glMap = build_submaps_visualization(opts);

            mrpt::opengl::Scene scene;
            scene.insert(glMap);

            scene.saveToFile(params_.debug_files_prefix + "_submaps_initial_pre.3Dscene"s);
        }

        save_current_key_frame_poses_as_tum(params_.debug_files_prefix + "_initial_pre_gnss.tum"s);

        // Run an initial LM pass to fit the GNSS measurements:
        optimize_graph();
    }

    save_current_key_frame_poses_as_tum(params_.debug_files_prefix + "_initial.tum"s);

    if (params_.save_submaps_viz_files)
    {  // Save viz of initial state:
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile(params_.debug_files_prefix + "_submaps_initial.3Dscene"s);
    }

    // Look for potential loop closures:
    // -----------------------------------------------
    // find next smallest potential loop closure?

    size_t accepted_lcs = 0;

    // index: <smallest_id, largest_id>
    std::set<std::pair<submap_id_t, submap_id_t>> alreadyChecked;
    // TODO: Consider a finer grade alreadyChecked reset?

    // repeat until checkedCount==0:
    // for (size_t lc_loop = 0; lc_loop < 1; lc_loop++)
    for (;;)
    {
        size_t checkedCount   = 0;
        bool   anyGraphChange = false;

        PotentialLoopOutput LCs = find_next_loop_closures(alreadyChecked);

        if (params_.max_number_lc_candidates > 0 && LCs.size() > params_.max_number_lc_candidates)
        {
            // dont shuffle: they are already sorted by expected score
#if 0
            std::random_device rd;
            std::mt19937       g(rd());
            std::shuffle(LCs.begin(), LCs.end(), g);
#endif
            LCs.resize(params_.max_number_lc_candidates);
        }

#if 0
        // Build a list of affected submaps, including how many times they
        // appear:
        std::map<submap_id_t, size_t> LC_submap_IDs_count;
        for (const auto& lc : LCs)
        {
            LC_submap_IDs_count[lc.smallest_id]++;
            LC_submap_IDs_count[lc.largest_id]++;
        }

        MRPT_LOG_INFO("List of submaps affected by LC candidates:");
        for (const auto& [id, n] : LC_submap_IDs_count)
        {
            MRPT_LOG_INFO_STREAM(
                "Submap #" << id << ": " << n << " candidate LCs.");
        }
#endif

        // check all LCs, and accept those that seem valid:
        for (size_t lcIdx = 0; lcIdx < LCs.size(); lcIdx++)
        {
            const auto& lc = LCs.at(lcIdx);

            auto IDs = std::make_pair(lc.smallest_id, lc.largest_id);

            if (alreadyChecked.count(IDs) != 0)
            {
                continue;
            }

            // a new pair. add it:
            alreadyChecked.insert(IDs);
            checkedCount++;

            MRPT_LOG_INFO_STREAM(
                "LC " << lcIdx << "/" << LCs.size() << ": " << lc.smallest_id << "<=>"
                      << lc.largest_id << " score: " << lc.score << " relPose="
                      << lc.relative_pose_largest_wrt_smallest.mean.asString() << " stds: "
                      << lc.relative_pose_largest_wrt_smallest.cov.asEigen()
                             .diagonal()
                             .array()
                             .sqrt()
                             .eval()
                             .transpose());

            const bool accepted = process_loop_candidate(lc);
            if (accepted)
            {
                anyGraphChange = true;
                accepted_lcs++;
            }
        }

        // free the RAM space of submaps not used any more:
        if (!LCs.empty())
        {
            for (submap_id_t id = 0; id < LCs.front().smallest_id; id++)
            {
                if (state_.submaps[id].local_map)
                {
                    MRPT_LOG_INFO_STREAM("Freeing memory for submap local map #" << id);
                    state_.submaps[id].local_map.reset();
                }
            }
        }

        // end of loop closures?
        if (!checkedCount)
        {
            break;  // no new LC was checked, we are done.
        }

        // any change to the graph? re-optimize it:
        if (anyGraphChange)
        {
            const auto largestDelta = optimize_graph();

            // re-visit all areas again
            if (largestDelta > params_.largest_delta_for_reconsider_all)
            {
                alreadyChecked.clear();
            }
        }
    }

    if (params_.save_submaps_viz_files)
    {  // Save viz of final state:
        VizOptions opts;
        auto       glMap = build_submaps_visualization(opts);

        mrpt::opengl::Scene scene;
        scene.insert(glMap);

        scene.saveToFile(params_.debug_files_prefix + "_submaps_final.3Dscene");
    }
    save_current_key_frame_poses_as_tum(params_.debug_files_prefix + "_final.tum"s);

    // At this point, we have optimized the KFs in state_.keyframesGraph.
    // Now, update all low-level keyframes in the simplemap:
    mrpt::maps::CSimpleMap outSM;

    for (size_t id = 0; id < sm.size(); id++)
    {
        const auto& [oldPose, sf, twist] = state_.sm->get(id);

        const auto& newKfGlobalPose = state_.kfGraph_get_pose(id);

        const auto newPose = mrpt::poses::CPose3DPDFGaussian::Create();
        newPose->mean      = newKfGlobalPose;
        newPose->cov.setIdentity();  // TODO! Get cov from optimizer?

        outSM.insert(newPose, sf, twist);
    }

    MRPT_LOG_INFO_STREAM("Overall number of accepted loop-closures: " << accepted_lcs);

    // Overwrite with new SM:
    sm = outSM;  // TODO: Implement CSimpleMap move ctor and use move() here
}

namespace
{
void make_pose_planar(mrpt::poses::CPose3D& p)
{
    p.z(0);
    p.setYawPitchRoll(p.yaw(), .0, .0);
}
void make_pose_planar_pdf(mrpt::poses::CPose3DPDFGaussian& pdf)
{
    make_pose_planar(pdf.mean);
    // z[2]=0, pitch[4]=0, roll[5]
    for (int i = 0; i < 6; i++)
    {
        pdf.cov(i, 2) = 0;
        pdf.cov(2, i) = 0;
        pdf.cov(i, 4) = 0;
        pdf.cov(4, i) = 0;
        pdf.cov(i, 5) = 0;
        pdf.cov(5, i) = 0;
    }
}
}  // namespace

void SimplemapLoopClosure::build_submap_from_kfs_into(
    const std::set<keyframe_id_t>& ids, SubMap& submap)
{
    using namespace mrpt::literals;  // _deg

    const keyframe_id_t refFrameId = *ids.begin();

    submap.kf_ids      = ids;
    submap.global_pose = keyframe_pose_in_simplemap(refFrameId);

    const auto invSubmapPose = -submap.global_pose;

    if (params_.assume_planar_world)
    {
        make_pose_planar(submap.global_pose);
    }

    MRPT_LOG_DEBUG_STREAM(
        "Defining submap #" << submap.id << " with " << ids.size() << " keyframes.");

    // Load the bbox of the frame from the SimpleMap metadata entry:
    // Insert all observations in this submap:

    std::optional<mrpt::math::TBoundingBox> bbox;
    size_t                                  gnssCount = 0;
    mrpt::maps::CSimpleMap                  subSM;  // aux SM for this submap

    for (const auto& id : submap.kf_ids)
    {
        const auto& [pose, sf, twist] = state_.sm->get(id);

        // Create submap SM, for latter use in GNSS geo-reference:
        auto relPdf = mrpt::poses::CPose3DPDFGaussian::Create();
        relPdf->copyFrom(*pose);
        relPdf->changeCoordinatesReference(invSubmapPose);

        subSM.insert(relPdf, sf, twist);

        // process metadata as embedded YAML "observation":
        if (auto oc = sf->getObservationByClass<mrpt::obs::CObservationComment>(); oc)
        {
            auto yml = mrpt::containers::yaml::FromText(oc->text);

            auto pMin = mrpt::math::TPoint3D::FromString(yml["frame_bbox_min"].as<std::string>());
            auto pMax = mrpt::math::TPoint3D::FromString(yml["frame_bbox_max"].as<std::string>());

            // transform bbox and extend bbox in local submap coordinates:
            const auto p = keyframe_relative_pose_in_simplemap(id, refFrameId);

            const auto pMinLoc = p.composePoint(pMin);
            const auto pMaxLoc = p.composePoint(pMax);
            if (!bbox)
            {
                bbox = mrpt::math::TBoundingBox::FromUnsortedPoints(pMinLoc, pMaxLoc);
            }
            else
            {
                bbox->updateWithPoint(pMinLoc);
                bbox->updateWithPoint(pMaxLoc);
            }
        }

        // Process GNSS?
        if (auto oG = sf->getObservationByClass<mrpt::obs::CObservationGPS>(); oG)
        {
            gnssCount++;
        }

    }  // for each keyframe

    // Try to generate geo-referencing data:
    if (params_.use_gnss && gnssCount > 2)
    {
        SMGeoReferencingParams geoParams;
        geoParams.fgParams.addHorizontalityConstraints = false;

        geoParams.logger            = this;
        geoParams.geodeticReference = state_.globalGeoRef;

        geoParams.fgParams.minimumUncertaintyXYZ = params_.gnss_minimum_uncertainty_xyz;

        auto geoResult = simplemap_georeference(subSM, geoParams);

        // decent solution?
        Eigen::Vector<double, 6> se3Stds = Eigen::Vector<double, 6>::Constant(100.0);

        if (!is_nullopt(geoResult.geo_ref))
        {
            se3Stds = deref(geoResult.geo_ref)
                          .T_enu_to_map.cov.asEigen()
                          .diagonal()
                          .array()
                          .sqrt()
                          .eval();
        }
        const auto angleStds = se3Stds.tail<3>();

        if (geoResult.final_rmse < 1.0)
        {
            // save in submap:

            // reset yaw/pitch/roll if they don't seem reliable:
            auto&                 p      = deref(geoResult.geo_ref).T_enu_to_map;
            std::array<double, 3> angles = {p.mean.yaw(), p.mean.pitch(), p.mean.roll()};
            for (int angleIdx = 0; angleIdx < 3; angleIdx++)
            {
                if (angleStds[angleIdx] > 0.5_deg)  // important threshold!
                {
                    angles[angleIdx] = angleIdx == 0 ?  //
                                           p.mean.asVectorVal()[3 + angleIdx]  // Yaw
                                                     :  //
                                           .0;  // pitch, roll

                    for (int i = 0; i < 6; i++)
                    {
                        p.cov(3 + angleIdx, i) = 0;
                        p.cov(i, 3 + angleIdx) = 0;
                    }
                    p.cov(3 + angleIdx, 3 + angleIdx) = mrpt::square(30.0_deg);
                }
            }
            p.mean.setYawPitchRoll(angles[0], angles[1], angles[2]);

            submap.geo_ref = geoResult.geo_ref;

            // Use one single global reference frame for all submaps:
            if (!state_.globalGeoRef && submap.geo_ref)
            {
                state_.globalGeoRef         = submap.geo_ref->geo_coord;
                state_.globalGeoRefSubmapId = submap.id;
            }

            // Update the global pose too:
            // T_0_i = (T_enu_0)⁻¹ · T_enu_i (notes picture! pass to paper)
            const auto T_enu_0 =
                state_.submaps.at(state_.globalGeoRefSubmapId).geo_ref->T_enu_to_map;
            auto T_0_enu = -T_enu_0;
            T_0_enu.cov.setZero();  // Ignore uncertainty of this first T

            const auto T_enu_i = submap.geo_ref->T_enu_to_map;
            const auto T_0_i   = T_0_enu + T_enu_i;

            MRPT_LOG_INFO_STREAM(
                "[build_submap_from_kfs_into] ACCEPTING submap #"
                << submap.id << " GNSS T_enu_to_map=" << deref(geoResult.geo_ref).T_enu_to_map.mean
                << "\n globalPose=" << T_0_i.mean  //
                << "\n was       =" << submap.global_pose << "\n se3Stds   =" << se3Stds.transpose()
                << "\n final_rmse=" << geoResult.final_rmse);

            submap.global_pose = T_0_i.getPoseMean();
        }
        else
        {
            MRPT_LOG_INFO_STREAM(
                "[build_submap_from_kfs_into] DISCARDING GNSS solution for "
                "submap #"
                << submap.id
                << "\n GNSS T_enu_to_map=" << deref(geoResult.geo_ref).T_enu_to_map.mean
                << "\n se3Stds=" << se3Stds.transpose()
                << "\n final_rmse=" << geoResult.final_rmse);
        }
    }

    if (bbox.has_value())
    {
        // Use bbox from SimpleMap metadata annotations:
        submap.bbox = *bbox;

        MRPT_LOG_DEBUG_STREAM(
            "[build_submap_from_kfs_into] Built bbox from metadata: " << bbox->asString());
    }
    else
    {
        // We have as input a simplemap without metadata.
        // Just build the whole metric map now:
        auto mm = get_submap_local_map(submap);
        mm.get();  // get future
    }

    // Fix zero-volume calculations for 2D lidar maps:
    if (bbox && std::abs(bbox->max.z - bbox->min.z) < 0.10)
    {
        bbox->max.z += 2.0;
        bbox->min.z -= 2.0;
        MRPT_LOG_DEBUG("[build_submap_from_kfs_into] Enlarging 2D bbox");
    }
}

mrpt::poses::CPose3D SimplemapLoopClosure::keyframe_pose_in_simplemap(keyframe_id_t kfId) const
{
    const auto& [pose, sf, twist] = state_.sm->get(kfId);
    ASSERT_(pose);
    return pose->getMeanVal();
}

mrpt::poses::CPose3D SimplemapLoopClosure::keyframe_relative_pose_in_simplemap(
    keyframe_id_t kfId, keyframe_id_t referenceKfId) const
{
    return keyframe_pose_in_simplemap(kfId) - keyframe_pose_in_simplemap(referenceKfId);
}

void SimplemapLoopClosure::updatePipelineDynamicVariablesForKeyframe(
    const keyframe_id_t id, const keyframe_id_t referenceId, const size_t threadIdx)
{
    auto& pts = state_.perThreadState_.at(threadIdx);

    auto& ps = pts.parameter_source;

    const auto& [globalPose, sf, twist] = state_.sm->get(id);

    // Set dynamic variables for twist usage within ICP pipelines
    // (e.g. de-skew methods)
    {
        mrpt::math::TTwist3D twistForIcpVars = {0, 0, 0, 0, 0, 0};
        if (twist)
        {
            twistForIcpVars = *twist;
        }

        ps.updateVariable("vx", twistForIcpVars.vx);
        ps.updateVariable("vy", twistForIcpVars.vy);
        ps.updateVariable("vz", twistForIcpVars.vz);
        ps.updateVariable("wx", twistForIcpVars.wx);
        ps.updateVariable("wy", twistForIcpVars.wy);
        ps.updateVariable("wz", twistForIcpVars.wz);
    }

    // robot pose:
    const auto p = keyframe_relative_pose_in_simplemap(id, referenceId);

    ps.updateVariable("robot_x", p.x());
    ps.updateVariable("robot_y", p.y());
    ps.updateVariable("robot_z", p.z());
    ps.updateVariable("robot_yaw", p.yaw());
    ps.updateVariable("robot_pitch", p.pitch());
    ps.updateVariable("robot_roll", p.roll());

    // pts.REL_POSE_SIGMA_XY;
    if (!pts.expr_threshold_sigma_final.is_compiled())
    {
        const std::map<std::string, double*> exprSymbols = {
            {"REL_POSE_SIGMA_XY", &pts.REL_POSE_SIGMA_XY}};

        pts.expr_threshold_sigma_final.register_symbol_table(exprSymbols);
        pts.expr_threshold_sigma_initial.register_symbol_table(exprSymbols);

        pts.expr_threshold_sigma_final.compile(
            params_.threshold_sigma_final, {}, "expr_threshold_sigma_final");

        pts.expr_threshold_sigma_initial.compile(
            params_.threshold_sigma_initial, {}, "expr_threshold_sigma_initial");
    }
    // Update:
    ps.updateVariable("REL_POSE_SIGMA_XY", pts.REL_POSE_SIGMA_XY);
    ps.updateVariable("SIGMA_INIT", pts.expr_threshold_sigma_initial.eval());
    ps.updateVariable("SIGMA_FINAL", pts.expr_threshold_sigma_final.eval());

    ps.updateVariable("ESTIMATED_SENSOR_MAX_RANGE", params_.max_sensor_range);

    // This will be overwritten by the actual ICP loop later on,
    // but we need to define all variables before building a local map:
    ps.updateVariable("ICP_ITERATION", 0);

    // Make all changes effective and evaluate the variables now:
    ps.realize();
}

mrpt::opengl::CSetOfObjects::Ptr SimplemapLoopClosure::build_submaps_visualization(
    const VizOptions& p) const
{
    auto glViz = mrpt::opengl::CSetOfObjects::Create();

    // Show graph:
    mrpt::containers::yaml extra_params;
    extra_params["show_ID_labels"] = true;
    extra_params["show_edges"]     = p.show_edges;

    auto glGraph = mrpt::opengl::graph_tools::graph_visualize(state_.submapsGraph, extra_params);

    // Show at an elevated height:
    glGraph->setLocation(0, 0, 10);

    glViz->insert(glGraph);

    // Boxes and mini-maps for each submap:
    for (const auto& [id, submap] : state_.submaps)
    {
        auto glSubmap = mrpt::opengl::CSetOfObjects::Create();

        if (p.show_bbox)
        {
            auto glBox = mrpt::opengl::CBox::Create();
            glBox->setWireframe(true);
            auto bbox = submap.bbox;
            glBox->setBoxCorners(bbox.min, bbox.max);
            glSubmap->insert(glBox);
        }
        if (!p.viz_point_layer.empty() && submap.local_map &&
            submap.local_map->layers.count(p.viz_point_layer) != 0)
        {
            auto&                  m = submap.local_map->layers.at(p.viz_point_layer);
            mp2p_icp::metric_map_t mm;
            mm.layers["dummy"] = m;

            mp2p_icp::render_params_t rp;
            rp.points.allLayers.pointSize = 3.0f;

            glSubmap->insert(mm.get_visualization(rp));
        }

        glSubmap->setPose(submap.global_pose);
        glViz->insert(glSubmap);
    }

    return glViz;
}

SimplemapLoopClosure::PotentialLoopOutput SimplemapLoopClosure::find_next_loop_closures(
    const std::set<std::pair<submap_id_t, submap_id_t>>& alreadyChecked) const
{
    using namespace std::string_literals;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "find_next_loop_closure");

    if (state_.submapsGraph.nodes.size() < 2)
    {
        return {};
    }

    struct InfoPerSubmap
    {
        mrpt::poses::CPose3DPDFGaussian pose;
        size_t                          depth = 0;
    };

    std::map<submap_id_t /*root id*/, std::multimap<double /*intersectRatio*/, PotentialLoop>>
        potentialLCs;

    // for debug files in SAVE_TREES only:
    static int tree_iter = 0;
    tree_iter++;

    // go on thru all nodes as root of Dijkstra:
    for (const auto& [root_id, _] : state_.submapsGraph.nodes)
    {
        mrpt::system::CTimeLoggerEntry tle1(profiler_, "find_next_loop_closure.single");

        mrpt::graphs::CDijkstra<typeof(state_.submapsGraph)> dijkstra(state_.submapsGraph, root_id);

        using tree_t =
            mrpt::graphs::CDirectedTree<const mrpt::graphs::CNetworkOfPoses3DCov::edge_t*>;

        const tree_t tree = dijkstra.getTreeGraph();

        std::map<submap_id_t, InfoPerSubmap> submapPoses;

        submapPoses[root_id] = {};  // perfect identity pose with zero cov.

        auto lambdaVisitTree = [&](mrpt::graphs::TNodeID const parent,
                                   const tree_t::TEdgeInfo& edgeToChild, size_t depthLevel)
        {
            auto& ips = submapPoses[edgeToChild.id];

            mrpt::poses::CPose3DPDFGaussian edge = *edgeToChild.data;

            if (edgeToChild.reverse)
            {
                edge = -edge;
            }

            if (params_.assume_planar_world)
            {
                make_pose_planar_pdf(edge);
            }

            ips.pose  = submapPoses[parent].pose + edge;
            ips.depth = depthLevel;

            MRPT_LOG_DEBUG_STREAM(
                "TREE visit: " << parent << " depth: " << depthLevel
                               << " pose mean=" << ips.pose.mean);
        };

        // run lambda:
        tree.visitDepthFirst(root_id, lambdaVisitTree);

        // Debug:
        if (SAVE_TREES)
        {
            const std::string d = "trees_"s + params_.debug_files_prefix;
            mrpt::system::createDirectory(d);
            const auto sFil = mrpt::format(
                "%s/tree_root_%04u_iter_%02i.3Dscene", d.c_str(), (unsigned int)root_id, tree_iter);
            std::cout << "[SAVE_TREES] Saving tree : " << sFil << "\n";

            mrpt::opengl::Scene scene;

            for (const auto& [id, m] : submapPoses)
            {
                {
                    auto glCorner = mrpt::opengl::stock_objects::CornerXYZSimple(2.5f);
                    glCorner->enableShowName();

                    std::string label = "#"s + std::to_string(id);
                    label += " d="s + std::to_string(m.depth);

                    if (state_.submaps.at(id).geo_ref.has_value())
                    {
                        label += " (WITH GPS)"s;
                    }

                    glCorner->setName(label);
                    glCorner->setPose(m.pose.mean);
                    scene.insert(glCorner);
                }

                {
                    auto glEllip = mrpt::opengl::CEllipsoid2D::Create();
                    glEllip->setCovMatrix(m.pose.cov.asEigen().block<2, 2>(0, 0));
                    glEllip->setLocation(m.pose.mean.translation());
                    glEllip->setQuantiles(2.0);
                    scene.insert(glEllip);
                }
            }

            scene.saveToFile(sFil);

        }  // end SAVE_TREES

        auto lambdaShrinkBbox = [](const mrpt::math::TBoundingBox& b) -> auto
        {
#if 0
            const double f = 0.75;

            auto r = b;
            r.min *= f;
            r.max *= f;
            return r;
#else
            return b;
#endif
        };

        // Look for all potential overlapping areas between root_id and all
        // other areas:

        const auto rootBbox = lambdaShrinkBbox(state_.submaps.at(root_id).bbox);

        for (const auto& [submapId, ips] : submapPoses)
        {
            // dont match against myself &
            // only analyze submaps IDs > root, to avoid duplicated checks:
            if (submapId <= root_id)
            {
                continue;
            }

            // we need at least topological distance>=2 for this to be L.C.
            // (except if we are using GNSS edges and one ID is the GNSS
            // reference submap!)
            if (ips.depth <= 1 &&
                (!state_.globalGeoRef.has_value() || (submapId != state_.globalGeoRefSubmapId &&
                                                      root_id != state_.globalGeoRefSubmapId)))
            {
                continue;  // skip it
            }

            const auto min_id = std::min<submap_id_t>(root_id, submapId);
            const auto max_id = std::max<submap_id_t>(root_id, submapId);

            // already checked?
            const auto IDs = std::make_pair(min_id, max_id);
            if (alreadyChecked.count(IDs) != 0)
            {
                continue;
            }

            // TODO(jlbc): finer approach without enlarging bboxes to their
            // global XYZ axis alined boxes:
            // Idea: run a small MonteCarlo run to estimate the likelihood
            // of an overlap for large uncertainties:
            const size_t MC_RUNS = mrpt::saturate_val<size_t>(10 * ips.depth, 50, 300);

            mrpt::poses::CPoseRandomSampler sampler;
            sampler.setPosePDF(ips.pose);

            if (PRINT_ALL_SCORES)
            {
                MRPT_LOG_INFO_STREAM(
                    "Relative pose: " << min_id << " <==> " << max_id << " pose: " << ips.pose);
            }

            double               bestScore = .0;
            mrpt::poses::CPose3D bestRelPose;

            const auto thisBBox = lambdaShrinkBbox(state_.submaps.at(submapId).bbox);

            for (size_t i = 0; i < MC_RUNS; i++)
            {
                mrpt::poses::CPose3D relPoseSample;
                sampler.drawSample(relPoseSample);

                const auto relativeBBox = thisBBox.compose(relPoseSample.asTPose());

                const auto bboxIntersect = rootBbox.intersection(relativeBBox);

                if (!bboxIntersect.has_value())
                {
                    continue;  // no overlap at all
                }

                const double intersectRatio =
                    bboxIntersect->volume() /
                    (0.5 * rootBbox.volume() + 0.5 * relativeBBox.volume());

                if (intersectRatio > bestScore)
                {
                    bestScore   = intersectRatio;
                    bestRelPose = relPoseSample;
                }
            }

            if (PRINT_ALL_SCORES)
            {
                MRPT_LOG_INFO_STREAM(
                    "Score for LC: " << min_id << " <==> " << max_id << " bestScore=" << bestScore
                                     << " topo_depth=" << ips.depth << " MC_RUNS=" << MC_RUNS);
            }

            if (bestScore < params_.min_volume_intersection_ratio_for_lc_candidate)
            {
                continue;
            }

            PotentialLoop lc;
            lc.largest_id           = max_id;
            lc.smallest_id          = min_id;
            lc.topological_distance = ips.depth;
            lc.score                = bestScore;
            if (root_id == min_id)
            {
                lc.relative_pose_largest_wrt_smallest = ips.pose;
            }
            else
            {  // inverse SE(3) relative pose
                ips.pose.inverse(lc.relative_pose_largest_wrt_smallest);
            }

            // for very long loops with too large uncertainty, replace the
            // (probably too bad) relative pose mean with the best one from
            // the MonteCarlo sample above:
            {
                const double std_xy = std::sqrt(ips.pose.cov.block(0, 0, 2, 2).determinant());

                const double submap_size = (thisBBox.max - thisBBox.min).norm();

                const double ratio = submap_size > 0 ? (std_xy / submap_size) : 1.0;

                if (PRINT_ALL_SCORES)
                {
                    MRPT_LOG_INFO_STREAM(
                        "|C(1:2,1:2)|=" << std_xy << " |submap_size|=" << submap_size
                                        << " ratio=" << ratio);
                }

                if (ratio > params_.min_lc_uncertainty_ratio_to_draw_several_samples)
                {
                    // Draw additional poses:
                    lc.draw_several_samples = true;
                }
            }

            potentialLCs[root_id].emplace(bestScore, lc);
        }

        // Do we have enough with this root_id submap?
        if (!potentialLCs[root_id].empty())
        {
            break;
        }

    }  // end for each root_id

    // debug, print potential LCs:
#if 0
    for (const auto& [score, lc] : potentialLCs)
    {
        MRPT_LOG_DEBUG_STREAM(
            "Initial potential LC: "
            << lc.smallest_id << " <==> " << lc.largest_id << " score=" << score
            << " topo_depth=" << lc.topological_distance);
    }
#endif

    // filter them, and keep the most promising ones, sorted by "score"
    PotentialLoopOutput result;
    for (const auto& [rootId, lcs] : potentialLCs)
    {
        const auto maxN            = params_.max_number_lc_candidates_per_submap;
        size_t     thisSubmapCount = 0;
        for (auto it = lcs.rbegin(); it != lcs.rend() && thisSubmapCount < maxN;
             ++it, ++thisSubmapCount)
        {
            const auto& [score, lc] = *it;
            result.push_back(lc);
        }
    }

    // debug, print potential LCs:
    for (const auto& lc : result)
    {
        MRPT_LOG_DEBUG_STREAM(
            "[find_lc] Potential LC: "  //
            << lc.smallest_id << " <==> " << lc.largest_id
            << " topo_depth=" << lc.topological_distance << " relPose: "
            << lc.relative_pose_largest_wrt_smallest.mean.asString() << " score: " << lc.score);
    }

    return result;
}

bool SimplemapLoopClosure::process_loop_candidate(const PotentialLoop& lc)
{
    using namespace std::string_literals;
    using namespace mrpt::literals;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "process_loop_candidate");

    // Apply ICP between the two submaps:
    const auto  idGlobal     = lc.smallest_id;
    const auto& submapGlobal = state_.submaps.at(idGlobal);

    const auto  idLocal     = lc.largest_id;
    const auto& submapLocal = state_.submaps.at(idLocal);

    auto mapGlobalFut = get_submap_local_map(submapGlobal);
    auto mapLocalFut  = get_submap_local_map(submapLocal);

    const auto& mapGlobal = mapGlobalFut.get();
    const auto& mapLocal  = mapLocalFut.get();

    ASSERT_(mapGlobal);
    ASSERT_(mapLocal);

    const auto& pcs_global = *mapGlobal;
    const auto& pcs_local  = *mapLocal;

    MRPT_LOG_DEBUG_STREAM("LC candidate: relPose=" << lc.relative_pose_largest_wrt_smallest);

    if (SAVE_LCS)
    {
        static int        nLoop = 0;
        const std::string d     = "lcs_"s + params_.debug_files_prefix;
        mrpt::system::createDirectory(d);
        const auto sDir = mrpt::format(
            "%s/loop_%04i_g%03u_l%03u", d.c_str(), nLoop++, (unsigned int)*pcs_global.id,
            (unsigned int)*pcs_local.id);
        mrpt::system::createDirectory(sDir);
        std::cout << "[LC] Saving loop closure files to: " << sDir << "\n";

        const bool ok1 = pcs_global.save_to_file(mrpt::system::pathJoin({sDir, "global.mm"}));
        const bool ok2 = pcs_local.save_to_file(mrpt::system::pathJoin({sDir, "local.mm"}));
        ASSERT_(ok1);
        ASSERT_(ok2);

        std::ofstream f(mrpt::system::pathJoin({sDir, "init_pose_local_wrt_global.txt"}));
        f << lc.relative_pose_largest_wrt_smallest;
    }

    const mrpt::math::TPose3D initGuess = lc.relative_pose_largest_wrt_smallest.mean.asTPose();

    const auto relPoseSigmaXY = std::sqrt(
        lc.relative_pose_largest_wrt_smallest.cov.asEigen().block<2, 2>(0, 0).determinant());

    bool atLeastOneGoodIcp = false;

    auto lambdaAddIcpEdge =
        [&](const mrpt::poses::CPose3DPDFGaussian& icpRelPose, const double icpQuality)
    {
        if (!state_.submapsGraph.edgeExists(idGlobal, idLocal))
        {
            state_.submapsGraph.insertEdge(idGlobal, idLocal, icpRelPose);
        }

        // and to the low-level graph too:
        const gtsam::Pose3 deltaPose = mrpt::gtsam_wrappers::toPose3(icpRelPose.mean);

        using gtsam::symbol_shorthand::X;

        // (1/2) Non-Robust edge for 1st PASS optimization, with "fake" cov
        double edge_std_xyz = 0.5;  // [m]
        double edge_std_ang = mrpt::DEG2RAD(0.5);

        // Use a variable variance depending on the ICP quality:
        ASSERT_(params_.icp_edge_worst_multiplier > 1.0);

        double std_multiplier =
            params_.icp_edge_worst_multiplier - (params_.icp_edge_worst_multiplier - 1.0) *
                                                    (icpQuality - params_.min_icp_goodness) /
                                                    params_.min_icp_goodness;

        edge_std_xyz *= std_multiplier;
        edge_std_ang *= std_multiplier;

        const double icp_edge_robust_param = params_.icp_edge_robust_param;

        gtsam::Vector6 sigmasNoRobust;
        sigmasNoRobust << edge_std_ang, edge_std_ang, edge_std_ang,  //
            edge_std_xyz, edge_std_xyz, edge_std_xyz;

        auto icpNoiseNoRubust = gtsam::noiseModel::Diagonal::Sigmas(sigmasNoRobust);

        // Non-robust graph:
        state_.kfGraphFG.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(*submapGlobal.kf_ids.begin()), X(*submapLocal.kf_ids.begin()), deltaPose,
            icpNoiseNoRubust);

        if (DEBUG_PRINT_BETWEEN_EDGES)
        {
            state_.kfGraphFG.back()->print("1/2 ICP edge factor: ");
        }

        // (2/2) Robust edge for 2nd PASS optimization, with real cov

        gtsam::Vector6 realSigmasXYZYPR = icpRelPose.cov.asEigen().diagonal().array().sqrt().eval();

        for (int i = 0; i < 3; i++)
        {
            realSigmasXYZYPR[3 + i] += params_.icp_edge_additional_noise_xyz;
            realSigmasXYZYPR[i] += mrpt::DEG2RAD(params_.icp_edge_additional_noise_ang_deg);
        }

        gtsam::Vector6 realSigmasGtsam;
        realSigmasGtsam << realSigmasXYZYPR[5], realSigmasXYZYPR[4], realSigmasXYZYPR[3],
            realSigmasXYZYPR[0], realSigmasXYZYPR[1], realSigmasXYZYPR[2];

        gtsam::noiseModel::Base::shared_ptr icpRobNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::GemanMcClure::Create(icp_edge_robust_param),
            gtsam::noiseModel::Diagonal::Sigmas(realSigmasGtsam));

        // Robust graph:
        state_.kfGraphFGRobust.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(*submapGlobal.kf_ids.begin()), X(*submapLocal.kf_ids.begin()), deltaPose,
            icpRobNoise);

        if (DEBUG_PRINT_BETWEEN_EDGES)
        {
            state_.kfGraphFGRobust.back()->print("2/2 ICP edge factor: ");
        }

        atLeastOneGoodIcp = true;
    };

    std::vector<mrpt::math::TPose3D> initGuesses;

    // 1) gross relocalization:
    if (params_.do_first_gross_relocalize)
    {
        // Build a reference map
        mp2p_icp::metric_map_t refMap;
        auto refPtsMap       = pcs_global.layers.at("points_to_register");  // "localmap"
        refMap.layers["raw"] = refPtsMap;

        // These options may be loaded from an INI file, etc.
        auto& likOpts =
            std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(refPtsMap)->likelihoodOptions;

        likOpts.max_corr_distance = 1.5;
        likOpts.decimation        = 1;
        likOpts.sigma_dist        = mrpt::square(0.2);  // variance

        // query observation:
        mrpt::obs::CSensoryFrame querySf;
        auto                     obs2 = mrpt::obs::CObservationPointCloud::Create();
        obs2->pointcloud              = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
            pcs_local.layers.at("points_to_register"));
        querySf.insert(obs2);

        mola::RelocalizationLikelihood_SE2::Input in;
        in.corner_min = mrpt::math::TPose2D(initGuess);
        in.corner_max = mrpt::math::TPose2D(initGuess);

        in.corner_min.x -= 10.0;
        in.corner_max.x += 10.0;

        in.corner_min.y -= 10.0;
        in.corner_max.y += 10.0;

        in.corner_min.phi -= 5.0_deg;
        in.corner_max.phi += 5.0_deg;

        in.observations   = querySf;
        in.reference_map  = refMap;
        in.resolution_xy  = 1.0;
        in.resolution_phi = 5.0_deg;

        MRPT_LOG_DEBUG_STREAM(
            "[Relocalize SE(2)] About to run with localPts="
            << obs2->pointcloud->size() << " globalPts=" << refMap.size()
            << " corner_min=" << in.corner_min << " corner_max=" << in.corner_max);

        const auto out = mola::RelocalizationLikelihood_SE2::run(in);

        // search top candidates:
        auto bestPoses = mola::find_best_poses_se2(out.likelihood_grid, 0.95);

        constexpr size_t MAX_BEST_POSES = 10;

        while (bestPoses.size() > MAX_BEST_POSES)
        {
            bestPoses.erase(--bestPoses.end());
        }

        MRPT_LOG_INFO_STREAM(
            "[Relocalize SE(2)] time_cost: " <<  //
            out.time_cost << " Top poses: " << bestPoses.size());

        for (const auto& [score, pose2D] : bestPoses)
        {
            auto pose  = mrpt::math::TPose3D(pose2D);
            pose.z     = initGuess.z;
            pose.roll  = initGuess.roll;
            pose.pitch = initGuess.pitch;
            initGuesses.push_back(pose);
        }

#if 0
    const auto nPhis = out.likelihood_grid.getSizePhi();
    std::vector<mrpt::math::CMatrixDouble> slices;
    slices.resize(nPhis);
    double maxLik = 0, minLik = 0;

    for (size_t iPhi = 0; iPhi < nPhis; iPhi++) {
      const double phi = out.likelihood_grid.idx2phi(iPhi);

      auto &s = slices.at(iPhi);
      out.likelihood_grid.getAsMatrix(phi, s);

      mrpt::keep_max(maxLik, s.maxCoeff());
      mrpt::keep_min(minLik, s.minCoeff());
    }

    // normalize all:
    for (size_t iPhi = 0; iPhi < nPhis; iPhi++) {
      auto &s = slices.at(iPhi);
      s -= minLik;
      s *= 1.0 / (maxLik - minLik);
    }

    // save:
    for (size_t iPhi = 0; iPhi < nPhis; iPhi++) {
      auto &s = slices.at(iPhi);

      s.saveToTextFile(mrpt::format("/tmp/slice_gid_%02u_lid_%02u_phi_%02u.txt",
                                    static_cast<unsigned int>(idGlobal),
                                    static_cast<unsigned int>(idLocal),
                                    static_cast<unsigned int>(iPhi)));

      mrpt::img::CImage im;
      im.setFromMatrix(s, true /*normalized [0,1]*/);
      bool savedOk = im.saveToFile(mrpt::format(
          "/tmp/slice_gid_%02u_lid_%02u_phi_%02u.png",
          static_cast<unsigned int>(idGlobal),
          static_cast<unsigned int>(idLocal), static_cast<unsigned int>(iPhi)));

      ASSERT_(savedOk);
    }

#endif
    }
    else if (params_.do_montecarlo_icp)
    {
        // Convert pose with uncertainty SE(3) -> SE(2)
        mrpt::poses::CPosePDFGaussian pdf_SE2;
        pdf_SE2.copyFrom(lc.relative_pose_largest_wrt_smallest);

        mola::RelocalizationICP_SE2::Input in;
        in.icp_minimum_quality = params_.min_icp_goodness;

        // Use multi-thread ICP:
        for (size_t threadIdx = 0; threadIdx < state_.perThreadState_.size(); threadIdx++)
        {
            auto& pts = state_.perThreadState_.at(threadIdx);

            // (this defines the local robot pose on the submap)
            updatePipelineDynamicVariablesForKeyframe(
                *submapLocal.kf_ids.begin(), *submapLocal.kf_ids.begin(), threadIdx);

            in.icp_pipeline.push_back(pts.icp);
        }

        // All threads with the same params:
        in.icp_parameters = params_.icp_parameters;

        double       std_x   = std::sqrt(pdf_SE2.cov(0, 0));
        double       std_y   = std::sqrt(pdf_SE2.cov(1, 1));
        const double std_yaw = std::sqrt(pdf_SE2.cov(2, 2));

        const double maxMapLenght = (submapLocal.bbox.max - submapLocal.bbox.min).norm();

        mrpt::saturate(std_x, 1.0, 0.25 * maxMapLenght);
        mrpt::saturate(std_y, 1.0, 0.25 * maxMapLenght);

        in.initial_guess_lattice.corner_min = {
            pdf_SE2.mean.x() - 3 * std_x,
            pdf_SE2.mean.y() - 3 * std_y,
            pdf_SE2.mean.phi() - 3 * std_yaw,
        };
        in.initial_guess_lattice.corner_max = {
            pdf_SE2.mean.x() + 3 * std_x,
            pdf_SE2.mean.y() + 3 * std_y,
            pdf_SE2.mean.phi() + 3 * std_yaw,
        };
        in.initial_guess_lattice.resolution_xy =
            mrpt::saturate_val<double>(std::max(std_x, std_y) * 3, 2.0, 0.2 * maxMapLenght);
        in.initial_guess_lattice.resolution_phi =
            mrpt::saturate_val<double>(std_yaw * 3, 10.0_deg, 30.0_deg);

        in.output_lattice.resolution_xyz   = 0.35;
        in.output_lattice.resolution_yaw   = 5.0_deg;
        in.output_lattice.resolution_pitch = 5.0_deg;
        in.output_lattice.resolution_roll  = 5.0_deg;

        in.reference_map = pcs_global;
        in.local_map     = pcs_local;

        in.on_progress_callback = [&](const mola::RelocalizationICP_SE2::ProgressFeedback& fb)
        {
            MRPT_LOG_INFO_STREAM(
                "[Relocalize SE(2)] Progress " << fb.current_cell << "/" << fb.total_cells);
        };

        MRPT_LOG_INFO_STREAM(
            "[Relocalize SE(2)] About to run with "
            << " corner_min=" << in.initial_guess_lattice.corner_min
            << " corner_max=" << in.initial_guess_lattice.corner_max);

        const auto out = mola::RelocalizationICP_SE2::run(in);

        // search top candidates:

        // Take the voxel with the largest number of poses:
        std::map<size_t, mrpt::math::TPose3D> bestVoxels;

        out.found_poses.visitAllVoxels(
            [&](const mola::HashedSetSE3::global_index3d_t&, const mola::HashedSetSE3::VoxelData& v)
            {
                if (v.poses().empty())
                {
                    return;
                }
                bestVoxels[v.poses().size()] = v.poses().front();
            });

        if (!bestVoxels.empty())
        {
            const auto& bestPose = bestVoxels.rbegin()->second;
#if 0
            lambdaAddIcpEdge(mrpt::poses::CPose3D(bestPose), 1.0);
            // ...
            return true;  // done
#endif
            MRPT_LOG_INFO_STREAM(
                "[Relocalize SE(2)] Result has "
                << out.found_poses.voxels().size() << " voxels, most populated one |V|="
                << bestVoxels.rbegin()->first << " bestPose: " << bestPose);

            // Let ICP to run again to recover the covariance.
            initGuesses.push_back(bestPose);
        }
    }

    // Default:
    if (initGuesses.empty())
    {
        // do not re-localize, just start with the first initial guess:
        initGuesses.push_back(initGuess);

        if (lc.draw_several_samples)
        {
            const auto sigmas = lc.relative_pose_largest_wrt_smallest.cov.asEigen()
                                    .diagonal()
                                    .array()
                                    .sqrt()
                                    .eval();

            const double std_x = sigmas[0] * 0.5;
            const double std_y = sigmas[1] * 0.5;
            // const double std_yaw = sigmas[3];

            for (int ix = -2; ix <= 2; ix += 1)
            {
                for (int iy = -2; iy <= 2; iy += 1)
                {
                    if (ix == 0 && iy == 0)
                    {
                        continue;  // center already added above
                    }

                    auto p = initGuess;
                    p.x += ix * std_x;
                    p.y += iy * std_y;
                    // p.yaw += rng.drawGaussian1D(.0, stdYaw);

                    initGuesses.push_back(p);
                }
            }

#if 0 
           for (size_t i = 0; i < 20; i++)
            {
                auto p = initGuess;
                p.x += rng.drawGaussian1D(.0, stdXY);
                p.y += rng.drawGaussian1D(.0, stdXY);
                p.yaw += rng.drawGaussian1D(.0, stdYaw);

                initGuesses.push_back(p);
            }
#endif
        }
    }

    // 2) refine with ICP:
    // Goal: keep all good results, let graph-slam to tell outliers.

    for (const auto& initPose : initGuesses)
    {
        mp2p_icp::Results icp_result;

        // Assume we are running this part in single thread (!)
        const size_t threadIdx = 0;

        auto& pts = state_.perThreadState_.at(threadIdx);

        pts.REL_POSE_SIGMA_XY = relPoseSigmaXY;  // before evaluating formulas

        // (this defines the local robot pose on the submap)
        updatePipelineDynamicVariablesForKeyframe(
            *submapLocal.kf_ids.begin(), *submapLocal.kf_ids.begin(), threadIdx);

        pts.icp->align(pcs_local, pcs_global, initPose, params_.icp_parameters, icp_result);

        MRPT_LOG_INFO_FMT(
            "ICP: goodness=%6.01f%% iters=%u pose=%s "
            "termReason=%s",
            100.0 * icp_result.quality, static_cast<unsigned int>(icp_result.nIterations),
            icp_result.optimal_tf.getMeanVal().asString().c_str(),
            mrpt::typemeta::enum2str(icp_result.terminationReason).c_str());

        // keep the best:
        if (icp_result.quality < params_.min_icp_goodness)
        {
            continue;
        }

        lambdaAddIcpEdge(icp_result.optimal_tf, icp_result.quality);
    }

    return atLeastOneGoodIcp;
}

mrpt::poses::CPose3D SimplemapLoopClosure::State::kfGraph_get_pose(const keyframe_id_t id) const
{
    using gtsam::symbol_shorthand::X;
    return mrpt::poses::CPose3D(
        mrpt::gtsam_wrappers::toTPose3D(kfGraphValues.at<gtsam::Pose3>(X(id))));
}

std::future<mp2p_icp::metric_map_t::Ptr> SimplemapLoopClosure::get_submap_local_map(
    const SubMap& submap)
{
    const size_t threadIdx = submap.id % state_.perThreadState_.size();

    auto fut = threads_.enqueue(
        [this, threadIdx](const SubMap* m)
        {
            // ensure only 1 thread is running for each per-thread data:
            auto lck = mrpt::lockHelper(state_.perThreadState_.at(threadIdx).mtx);

            auto mm = this->impl_get_submap_local_map(*m);

            return mm;
        },
        &submap);
    return fut;
}

mp2p_icp::metric_map_t::Ptr SimplemapLoopClosure::impl_get_submap_local_map(const SubMap& submap)
{
    if (submap.local_map)
    {
        return submap.local_map;
    }

    const size_t threadIdx = submap.id % state_.perThreadState_.size();

    const keyframe_id_t refFrameId = *submap.kf_ids.begin();

    auto& pts = state_.perThreadState_.at(threadIdx);

    MRPT_TODO("Use mp2p_icp::update_velocity_buffer_from_obs() once mp2p_icp 2.5.0 is available");
    const auto lambdaProcessLocalVelocityBuffer = [&](const mrpt::obs::CObservation::Ptr& obs)
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
            pts.parameter_source.localVelocityBuffer.fromYAML(lvb);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing 'local_velocity_buffer': " << e.what() << "\n";
            return;
        }
    };

    // Insert all observations in this submap:
    for (const auto& id : submap.kf_ids)
    {
        // (this defines the local robot pose on the submap)
        updatePipelineDynamicVariablesForKeyframe(id, refFrameId, threadIdx);

        // now that we have valid dynamic variables, check if we need to
        // create the submap on the first KF:
        if (!submap.local_map)
        {
            // Create empty local map:
            submap.local_map = mp2p_icp::metric_map_t::Create();

            // and populate with empty metric maps:
            pts.parameter_source.realize();

            mrpt::obs::CSensoryFrame dummySF;
            {
                auto obs = mrpt::obs::CObservationPointCloud::Create();
                dummySF.insert(obs);
            }

            mp2p_icp_filters::apply_generators(
                pts.local_map_generators, dummySF, *submap.local_map);
        }

        // insert observations from the keyframe:
        // -------------------------------------------

        // Extract points from observation:
        auto observation = mp2p_icp::metric_map_t::Create();

        const auto& [pose, sf, twist] = state_.sm->get(id);

        // Some frames may be empty:
        if (sf->empty())
        {
            continue;
        }
        if (sf->size() == 1 &&
            IS_CLASS(*sf->getObservationByIndex(0), mrpt::obs::CObservationComment))
        {
            continue;
        }

        // First, search for velocity buffer data:
        for (const auto& obs : *sf)
        {
            ASSERT_(obs);
            lambdaProcessLocalVelocityBuffer(obs);
        }

        // Next, do the actual sensor data processing:

        mrpt::system::CTimeLoggerEntry tle0(profiler_, "add_submap_from_kfs.apply_generators");

        for (const auto& o : *sf)
        {
            mp2p_icp_filters::apply_generators(pts.obs_generators, *o, *observation);
        }

        tle0.stop();

        // Filter/segment the point cloud (optional, but normally will be
        // present):
        mrpt::system::CTimeLoggerEntry tle1(profiler_, "add_submap_from_kfs.filter_pointclouds");

        mp2p_icp_filters::apply_filter_pipeline(pts.pc_filter, *observation, profiler_);

        tle1.stop();

        // Merge "observation_layers_to_merge_local_map" in local map:
        // ---------------------------------------------------------------
        mrpt::system::CTimeLoggerEntry tle3(profiler_, "add_submap_from_kfs.update_local_map");

        // Input  metric_map_t: observation
        // Output metric_map_t: state_.local_map

        // 1/4: temporarily make a (shallow) copy of the observation layers
        // into the local map:
        ASSERT_(submap.local_map);
        for (const auto& [lyName, lyMap] : observation->layers)
        {
            ASSERTMSG_(
                submap.local_map->layers.count(lyName) == 0,
                mrpt::format(
                    "Error: local map layer name '%s' collides with one of "
                    "the observation layers, please use different layer "
                    "names.",
                    lyName.c_str()));

            submap.local_map->layers[lyName] = lyMap;  // shallow copy
        }

        // 2/4: Make sure dynamic variables are up-to-date,
        // in particular, [robot_x, ..., robot_roll]
        // already done above: updatePipelineDynamicVariables();

        // 3/4: Apply pipeline
        mp2p_icp_filters::apply_filter_pipeline(pts.obs2map_merge, *submap.local_map, profiler_);

        // 4/4: remove temporary layers:
        for (const auto& [lyName, lyMap] : observation->layers)
        {
            submap.local_map->layers.erase(lyName);
        }

        tle3.stop();
    }  // end for each keyframe ID

    mp2p_icp_filters::apply_filter_pipeline(pts.submap_final_filter, *submap.local_map, profiler_);

    // add metadata to local map (for generated debug .icplog files):
    submap.local_map->label = params_.debug_files_prefix;
    submap.local_map->id    = submap.id;

    // Add geo-referencing, if it exists:
    if (submap.geo_ref)
    {
        submap.local_map->georeferencing = submap.geo_ref;
    }

    // Actual bbox: from point cloud layer:
    std::optional<mrpt::math::TBoundingBoxf> theBBox;

    for (const auto& [name, map] : submap.local_map->layers)
    {
        const auto* ptsMap = mp2p_icp::MapToPointsMap(*map);
        if (!ptsMap || ptsMap->empty())
        {
            continue;
        }

        auto bbox = ptsMap->boundingBox();
        if (!theBBox)
        {
            theBBox = bbox;
        }
        else
        {
            theBBox = theBBox->unionWith(bbox);
        }
    }

    std::stringstream debugInfo;
    debugInfo << "submap #" << submap.id << " with " << submap.kf_ids.size()
              << " KFs, local_map: " << submap.local_map->contents_summary();

    if (!theBBox.has_value())
    {
        THROW_EXCEPTION_FMT("no map bbox (!): %s", debugInfo.str().c_str());
    }

    submap.bbox.min = theBBox->min.cast<double>();
    submap.bbox.max = theBBox->max.cast<double>();

    MRPT_LOG_DEBUG_STREAM("Done. Submap metric map: " << debugInfo.str());

    return submap.local_map;
}

double SimplemapLoopClosure::optimize_graph()
{
    // low-level KF graph:
    auto lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();
    // auto lmParams = gtsam::GaussNewtonParams();

    // Pass 1
    const double errorInit1 = state_.kfGraphFG.error(state_.kfGraphValues);
    const double rmseInit1  = std::sqrt(errorInit1 / static_cast<double>(state_.kfGraphFG.size()));

    gtsam::LevenbergMarquardtOptimizer lm1(state_.kfGraphFG, state_.kfGraphValues, lmParams);

    const auto& optimalValues1 = lm1.optimize();

    const double errorEnd1 = state_.kfGraphFG.error(optimalValues1);
    const double rmseEnd1  = std::sqrt(errorEnd1 / static_cast<double>(state_.kfGraphFG.size()));

    // Pass 2
    const double errorInit2 = state_.kfGraphFGRobust.error(optimalValues1);
    const double rmseInit2 =
        std::sqrt(errorInit2 / static_cast<double>(state_.kfGraphFGRobust.size()));

    gtsam::LevenbergMarquardtOptimizer lm2(state_.kfGraphFGRobust, optimalValues1, lmParams);
    const auto&                        optimalValues2 = lm2.optimize();

    state_.kfGraphValues = optimalValues2;

    const double errorEnd2 = state_.kfGraphFGRobust.error(optimalValues2);
    const double rmseEnd2 =
        std::sqrt(errorEnd2 / static_cast<double>(state_.kfGraphFGRobust.size()));

    // Update submaps global pose:
    double largestDelta = 0;

    for (auto& [submapId, submap] : state_.submaps)
    {
        const auto  refId      = *submap.kf_ids.begin();
        const auto& newPose    = state_.kfGraph_get_pose(refId);
        auto&       targetPose = submap.global_pose;
        const auto  deltaPose  = (targetPose - newPose).translation().norm();
        mrpt::keep_max(largestDelta, deltaPose);

        MRPT_LOG_DEBUG_STREAM(
            "Optimized refPose of submap #" << submapId << ":\n old=" << targetPose.asTPose()
                                            << "\n new=" << newPose.asTPose());

        // 1) in map<> data structure:
        targetPose = newPose;

        // 2) and in mrpt graph nodes:
        state_.submapsGraph.nodes.at(submapId) = newPose;
    }

    auto bckCol =
        mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO);
    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) =
        mrpt::system::ConsoleForegroundColor::BRIGHT_GREEN;
    MRPT_LOG_INFO_STREAM(
        "***** Graph re-optimized in "
        << lm1.iterations() << "/" << lm2.iterations() << " iters, RMSE: 1st PASS:" << rmseInit1
        << " ==> " << rmseEnd1 << " / 2nd PASS: " << rmseInit2 << " ==> " << rmseEnd2
        << " largestDelta=" << largestDelta << " [m]");
    mrpt::system::COutputLogger::logging_levels_to_colors().at(mrpt::system::LVL_INFO) = bckCol;

    if (PRINT_FG_ERRORS)
    {
        const double errorPrintThres = 10.0;

        state_.kfGraphFG.printErrors(
            optimalValues1, "================ 1ST PASS Factor errors ============\n",
            gtsam::DefaultKeyFormatter,
            std::function<bool(const gtsam::Factor*, double whitenedError, size_t)>(
                [&](const gtsam::Factor* /*f*/, double error, size_t /*index*/)
                { return error > errorPrintThres; }));

        state_.kfGraphFGRobust.printErrors(
            optimalValues2, "================ 2ND PASS Factor errors ============\n",
            gtsam::DefaultKeyFormatter,
            std::function<bool(const gtsam::Factor*, double whitenedError, size_t)>(
                [&](const gtsam::Factor* /*f*/, double error, size_t /*index*/)
                { return error > errorPrintThres; }));
    }

    return largestDelta;
}

std::vector<std::set<SimplemapLoopClosure::keyframe_id_t>> SimplemapLoopClosure::detect_sub_maps()
    const
{
    std::vector<std::set<keyframe_id_t>>   detectedSubMaps;
    std::set<keyframe_id_t>                pendingKFs;
    double                                 pendingKFsAccumDistance = 0;
    std::optional<mrpt::poses::CPose3D>    lastPose;
    std::optional<mrpt::Clock::time_point> lastTime;

    bool anyValidObsInPendingSet = false;

    ASSERT_(state_.sm);

    const auto& sm = *state_.sm;

    const auto   bbox     = SimpleMapBoundingBox(sm);
    const double smLength = (bbox.max - bbox.min).norm();

    const double max_submap_length = mrpt::saturate_val(
        params_.submap_max_length_wrt_map * smLength, params_.submap_min_absolute_length,
        params_.submap_max_absolute_length);

    MRPT_LOG_INFO_FMT("Using submap length=%.02f m", max_submap_length);

    for (size_t i = 0; i < sm.size(); i++)
    {
        pendingKFs.insert(i);

        const auto pose_i_local = keyframe_relative_pose_in_simplemap(i, *pendingKFs.begin());

        const auto& [pose_i, sf_i, twist_i] = state_.sm->get(i);

        // don't cut a submap while we are processing empty SFs since we
        // don't know for how long it will take and we might end up with a
        // totally empty final submap
        if (!sf_has_real_mapping_observations(*sf_i))
        {
            continue;
        }
        anyValidObsInPendingSet = true;

        mrpt::poses::CPose3D incrPose;
        if (lastPose)
        {
            incrPose = pose_i_local.getPoseMean() - *lastPose;
        }
        lastPose = pose_i_local.getPoseMean();

        pendingKFsAccumDistance += incrPose.translation().norm();

        double                                       time_since_last_kf = 0;
        const std::optional<mrpt::Clock::time_point> thisTime           = sf_timestamp(*sf_i);

        if (lastTime && thisTime)
        {
            time_since_last_kf = mrpt::system::timeDifference(*lastTime, *thisTime);
        }

        if (!lastTime && thisTime)
        {
            lastTime = *thisTime;
        }

        if (pendingKFsAccumDistance >= max_submap_length ||
            time_since_last_kf > params_.max_time_between_kfs_to_break_submap)
        {
            detectedSubMaps.emplace_back(pendingKFs);
            pendingKFs.clear();
            lastTime.reset();
            pendingKFsAccumDistance = 0;
            anyValidObsInPendingSet = false;
        }
    }
    // remaining ones?
    if (!pendingKFs.empty())
    {
        if (anyValidObsInPendingSet)
        {
            detectedSubMaps.emplace_back(pendingKFs);
        }
        else
        {
            // just append to the last submap, since none of the SFs has
            // data to build a new local map
            for (const auto id : pendingKFs)
            {
                detectedSubMaps.back().insert(id);
            }
        }
    }

    return detectedSubMaps;
}

void SimplemapLoopClosure::save_current_key_frame_poses_as_tum(const std::string& outTumFile) const
{
    ASSERT_(state_.sm);

    mrpt::poses::CPose3DInterpolator path;

    for (size_t id = 0; id < state_.sm->size(); id++)
    {
        const auto& [oldPose, sf, twist] = state_.sm->get(id);

        const auto& newKfGlobalPose = state_.kfGraph_get_pose(id);

        ASSERT_(!sf->empty());
        const auto t = sf->getObservationByIndex(0)->timestamp;

        path.insert(t, newKfGlobalPose);
    }

    path.saveToTextFile_TUM(outTumFile);
}
