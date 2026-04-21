/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/far_planner.h"

/***************************************************************************************/
FARMaster::FARMaster()
{
   /* initialize node */
  nh_ = rclcpp::Node::make_shared("far_planner_node");
  
  /* initialize transform buffer and listener */
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Node Initiated");
}

void FARMaster::Init() {
  /* initialize subscriber and publisher */
  reset_graph_sub_    = nh_->create_subscription<std_msgs::msg::Empty>("/reset_visibility_graph", 5, std::bind(&FARMaster::ResetGraphCallBack, this, std::placeholders::_1));
  odom_sub_           = nh_->create_subscription<nav_msgs::msg::Odometry>("/odom_world", 5, std::bind(&FARMaster::OdomCallBack, this, std::placeholders::_1));
  terrain_sub_        = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_cloud", 1, std::bind(&FARMaster::TerrainCallBack, this, std::placeholders::_1));
  scan_sub_           = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("/scan_cloud", 5, std::bind(&FARMaster::ScanCallBack, this, std::placeholders::_1));
  waypoint_sub_       = nh_->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 1, std::bind(&FARMaster::WaypointCallBack, this, std::placeholders::_1));
  terrain_local_sub_  = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_local_cloud", 1, std::bind(&FARMaster::TerrainLocalCallBack, this, std::placeholders::_1));
  joy_command_sub_    = nh_->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, std::bind(&FARMaster::JoyCommandCallBack, this, std::placeholders::_1));
  update_command_sub_ = nh_->create_subscription<std_msgs::msg::Bool>("/update_visibility_graph", 5, std::bind(&FARMaster::UpdateCommandCallBack, this, std::placeholders::_1));
  goal_pub_           = nh_->create_publisher<geometry_msgs::msg::PointStamped>("/way_point",5);
  boundary_pub_       = nh_->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary",5);

  // Timers
  runtime_pub_        = nh_->create_publisher<std_msgs::msg::Float32>("/runtime",1);
  planning_time_pub_  = nh_->create_publisher<std_msgs::msg::Float32>("/planning_time",1);
  traverse_time_pub_  = nh_->create_publisher<std_msgs::msg::Float32>("/far_traverse_time", 5);

  // planning status publisher
  reach_goal_pub_     = nh_->create_publisher<std_msgs::msg::Bool>("/far_reach_goal_status", 5);

  // Terminal formatting subscriber
  read_command_sub_   = nh_->create_subscription<std_msgs::msg::String>("/read_file_dir", 1, std::bind(&FARMaster::ReadFileCommand, this, std::placeholders::_1));
  save_command_sub_   = nh_->create_subscription<std_msgs::msg::String>("/save_file_dir", 1, std::bind(&FARMaster::SaveFileCommand, this, std::placeholders::_1));

  // Services
  start_far_planner_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      "/start_far_planner",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Gate the planning loop; v-graph updates can still be controlled separately.
        if (!is_init_completed_) {
          response->success = false;
          response->message = "FAR Planner not initialized yet.";
          return;
        }

        is_system_started_ = true;
        response->success = true;
        response->message = "FAR Planner system started (planning enabled).";
        RCLCPP_WARN(nh_->get_logger(), "FARMaster: Start FAR planner system (service call).");
      });

  stop_far_planner_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      "/stop_far_planner",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Disable planning AND command robot to stop.
        is_system_started_ = false;

        goal_waypoint_stamped_.header.stamp = nh_->now();
        goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
        goal_pub_->publish(goal_waypoint_stamped_);
        NodePtrStack empty_path;
        planner_viz_.VizPath(empty_path);
        is_planner_running_ = false;
        nav_heading_ = Point3D(0, 0, 0);

        response->success = true;
        response->message = "FAR Planner system stopped (planning disabled; robot stop waypoint published).";
        RCLCPP_WARN(nh_->get_logger(), "FARMaster: Stop FAR planner system (service call).");
      });

  stop_vgraph_update_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      "/stop_visibility_graph_update",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        is_stop_update_ = true;
        response->success = true;
        response->message = "Visibility graph updates stopped (frozen).";
        RCLCPP_WARN(nh_->get_logger(), "FARMaster: Stop visibility graph update (service call).");
      });

  resume_vgraph_update_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      "/resume_visibility_graph_update",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Resume dynamic V-graph updates
        is_stop_update_ = false;
        // Force a refresh cycle so it updates right after the request.
        is_graph_init_ = false;

        response->success = true;
        response->message = "Visibility graph updates resumed and refresh requested.";
        RCLCPP_WARN(nh_->get_logger(), "FARMaster: Resume visibility graph update (service call). Rebuilding...");
      });

  load_vgraph_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      "/load_visibility_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Uses the same file path as auto-load: ROS parameter "vgraph_file_path".
        // This keeps the service interface simple (no custom srv) and works well with RViz.
        std::string filename = "";
        (void)nh_->get_parameter("vgraph_file_path", filename);

        if (filename.empty()) {
          response->success = false;
          response->message = "Missing parameter vgraph_file_path (cannot load visibility graph).";
          RCLCPP_ERROR(nh_->get_logger(), "FARMaster: /load_visibility_graph called but vgraph_file_path is empty.");
          return;
        }

        response->success = true;
        response->message = std::string("Loading visibility graph from: ") + filename;
        RCLCPP_WARN(nh_->get_logger(), "FARMaster: Load visibility graph (service call): %s", filename.c_str());
        this->LoadVisibilityGraph(filename);
      });

  save_vgraph_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      "/save_visibility_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string filename = "";
        (void)nh_->get_parameter("vgraph_file_path", filename);

        if (filename.empty()) {
          response->success = false;
          response->message = "Missing parameter vgraph_file_path (cannot save visibility graph).";
          RCLCPP_ERROR(nh_->get_logger(), "FARMaster: /save_visibility_graph called but vgraph_file_path is empty.");
          return;
        }

        response->success = true;
        response->message = std::string("Saving visibility graph to: ") + filename;
        RCLCPP_WARN(nh_->get_logger(), "FARMaster: Save visibility graph (service call): %s", filename.c_str());
        this->SaveVisibilityGraph(filename);
      });

  // DEBUG Publisher
  dynamic_obs_pub_     = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_dynamic_obs_debug",1);
  surround_free_debug_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_free_debug",1);
  surround_obs_debug_  = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_obs_debug",1);
  scan_grid_debug_     = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_scanGrid_debug",1);
  new_PCL_pub_         = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_new_debug",1);
  terrain_height_pub_  = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_terrain_height_debug",1);

  //print publisher and subscriber init complete
  // RCLCPP_INFO(nh_->get_logger(), "FAR Planner Subscriber and Publisher Initiated");

  // Initialize pending graph load flags BEFORE LoadROSParams, which may set them
  is_pending_graph_load_ = false;
  pending_graph_load_path_ = "";

  this->LoadROSParams();

  //print ROS params load complete
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner ROS Params Initiated");

  /*init path generation thred callback*/
  const float duration_time = 1.0f / master_params_.main_run_freq;
  main_event_     = nh_->create_wall_timer(std::chrono::milliseconds(int(duration_time * 1000)), std::bind(&FARMaster::MainLoopCallBack, this));
  planning_event_ = nh_->create_wall_timer(std::chrono::milliseconds(int(duration_time * 1000)), std::bind(&FARMaster::PlanningCallBack, this));

  //print callback init complete
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Callback Initiated");

  /* init Dynamic Planner Processing Objects */
  contour_detector_.Init(cdetect_params_);
  graph_manager_.Init(nh_, graph_params_);
  graph_planner_.Init(nh_, gp_params_);
  contour_graph_.Init(nh_, cg_params_);
  planner_viz_.Init(nh_);
  map_handler_.Init(map_params_);
  scan_handler_.Init(scan_params_);
  graph_msger_.Init(nh_, msger_parmas_);

  //print processing objects init complete
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Processing Objects Initiated");

  /* init internal params */
  odom_node_ptr_      = NULL;
  is_cloud_init_      = false;
  is_odom_init_       = false;
  is_scan_init_       = false;
  is_planner_running_ = false;
  is_graph_init_      = false;
  is_reset_env_       = false;
  // Default behavior: keep the v-graph frozen until explicitly resumed (checkbox/service/topic)
  // so the system can load a saved VGH and plan with it unchanged.
  is_stop_update_     = true;
  is_init_completed_  = false;
  // Default behavior: planning stays disabled until explicitly started.
  is_system_started_  = false;
  // NOTE: is_pending_graph_load_ and pending_graph_load_path_ are NOT reset here
  // because they are set by LoadROSParams() which runs BEFORE this block.
  // Resetting them here would overwrite the auto-load configuration.

  // allocate memory to pointers
  new_vertices_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_obs_ptr_         = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_free_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_cloud_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  scan_grid_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  local_terrain_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  terrain_height_ptr_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  viewpoint_around_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  kdtree_viewpoint_obs_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());

  // set kdtree sorted value
  FARUtil::kdtree_new_cloud_->setSortedResults(false);
  FARUtil::kdtree_filter_cloud_->setSortedResults(false);
  kdtree_viewpoint_obs_cloud_->setSortedResults(false);

  // init global utility cloud
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();

  // clear temp vectors and memory
  this->ClearTempMemory();
  FARUtil::robot_pos = Point3D(0,0,0);
  FARUtil::free_odom_p = Point3D(0,0,0);

  robot_pos_   = Point3D(0,0,0);
  nav_heading_ = Point3D(0,0,0);
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;

  // waiting for one second
  std::this_thread::sleep_for(std::chrono::seconds(1));

  printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
  std::cout<<std::endl;
  if (master_params_.is_static_env) {
    std::cout<<"\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"<<std::endl;
  } else {
    std::cout<< "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n" << std::endl;
  }
  std::cout<<"\n"<<std::endl;

  // init complete
  is_init_completed_ = true;
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Initiated Complete");

  // Load visibility graph immediately if auto-load is enabled (no need to wait for odom)
  if (is_pending_graph_load_ && !pending_graph_load_path_.empty()) {
    RCLCPP_WARN(nh_->get_logger(), "=== VGRAPH: Loading graph immediately from: %s ===", pending_graph_load_path_.c_str());
    this->LoadVisibilityGraph(pending_graph_load_path_);
    is_pending_graph_load_ = false;
    pending_graph_load_path_ = "";
  }
}

void FARMaster::ResetEnvironmentAndGraph() {
  this->ResetInternalValues();
  if (!FARUtil::IsDebug) { // Terminal Output
    printf("\033[A"), printf("\033[A"), printf("\033[2K");
    std::cout<< "\033[1;31m V-Graph Resetting...\033[0m\n" << std::endl;
  }
  graph_manager_.ResetCurrentGraph();
  map_handler_.ResetGripMapCloud();
  graph_planner_.ResetPlannerInternalValues();
  contour_graph_.ResetCurrentContour();
  /* Reset clouds */
  FARUtil::surround_obs_cloud_->clear();
  FARUtil::surround_free_cloud_->clear();
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();
  FARUtil::cur_new_cloud_->clear();
  FARUtil::cur_dyobs_cloud_->clear();
  /* Stop the robot if it is moving */
  goal_waypoint_stamped_.header.stamp = nh_->now();
  goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_->publish(goal_waypoint_stamped_);
  NodePtrStack empty_path;
  planner_viz_.VizPath(empty_path);
}

void FARMaster::MainLoopCallBack() {
  if (!is_init_completed_) {
    return;
  }

  if (is_reset_env_) {
      this->ResetEnvironmentAndGraph();
      is_reset_env_ = false;
      if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "****************** Graph and Env Reset ******************");
      return;
  }

  if (!this->PreconditionCheck()) {
      // Even if preconditions aren't met, keep visualizing the loaded graph
      if (is_graph_init_ && !nav_graph_.empty()) {
        planner_viz_.VizGraph(nav_graph_);
        planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);
      }
      return;
  }

  /* add main process after this line */
  graph_manager_.UpdateRobotPosition(robot_pos_);
  odom_node_ptr_ = graph_manager_.GetOdomNode();
  if (odom_node_ptr_ == NULL) {
    RCLCPP_WARN(nh_->get_logger(),"FAR: Waiting for Odometry...");
    return;
  }
  /* Extract Vertices and new nodes */
  FARUtil::Timer.start_time("Total V-Graph Update");
  contour_detector_.BuildTerrainImgAndExtractContour(odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
  contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);
  if (is_graph_init_) {
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Local V-Graph Updated. Number of local vertices: "<<ContourGraph::contour_graph_.size()<<std::endl;
  }
  /* Adjust heights with terrain */
  map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);
  map_handler_.AdjustNodesHeight(nav_graph_);
  // Truncate for local range nodes
  graph_manager_.UpdateGlobalNearNodes();
  near_nav_graph_ = graph_manager_.GetExtendLocalNode();
  // Match near nav nodes with contour
  contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_);
  if (master_params_.is_visual_opencv) {
    FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
    cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
    contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
  }
  /* update planner graph */
  new_nodes_.clear();
  if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
    new_nodes_ = graph_manager_.GetNewNodes();
  }
  if (is_graph_init_) {
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<< "Number of new vertices adding to global V-Graph: "<< new_nodes_.size()<<std::endl;
  }
  /* Graph Updating */
  graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);

  runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_) / 1000.f; // Unit: second
  // runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_); // Unit: ms
  runtime_pub_->publish(runtimer_);

  /* Update v-graph in other modules */
  nav_graph_ = graph_manager_.GetNavGraph();
  if (is_graph_init_) {
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Global V-Graph Updated. Number of global vertices: "<<nav_graph_.size()<<std::endl;
  }
  contour_graph_.ExtractGlobalContours();      // Global Polygon Update
  graph_planner_.UpdaetVGraph(nav_graph_);     // Graph Planner Update
  graph_msger_.UpdateGlobalGraph(nav_graph_);  // Graph Messager Update

  /* Publish local boundary to lower level local planner */
  this->LocalBoundaryHandler(ContourGraph::local_boundary_);

  /* Viz Navigation Graph */
  const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
  if (last_internav_ptr != NULL) {
    planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
  }
  planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
  planner_viz_.VizNodes(graph_manager_.GetOutContourNodes(), "out_contour", VizColor::YELLOW);
  planner_viz_.VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
  planner_viz_.VizGraph(nav_graph_);
  planner_viz_.VizContourGraph(ContourGraph::contour_graph_);
  planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);

  // publish nodes visualization
  planner_viz_.PubNodesVisualization();

  if (is_graph_init_) { 
    if (FARUtil::IsDebug) {
      std::cout<<" ========================================================== "<<std::endl;
    } else { // cleanup outputs in terminal
      for (int i = 0; i < 6; i++) {
        printf("\033[A");
      }
    }
  }

  if (!is_graph_init_ && !nav_graph_.empty()) {
    is_graph_init_ = true;
    printf("\033[A"), printf("\033[A"), printf("\033[2K");
    std::cout<< "\033[1;32m V-Graph Initialized \033[0m\n" << std::endl;
  }

}

void FARMaster::PlanningCallBack() {
  if (!is_init_completed_ || !is_graph_init_ || !is_system_started_) return;
  const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
  if (goal_ptr == NULL) {
    /* Graph Traversablity Update */
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Adding Goal to V-Graph "<<"Time: "<<0.f<<"ms"<<std::endl;
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Path Search "<<"Time: "<<0.f<<"ms"<<std::endl;
  } else { 
    // Update goal postion with nearby terrain cloud
    const Point3D ori_p = graph_planner_.GetOriginNodePos(true);
    PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
    PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
    map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);
    map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);
    graph_planner_.UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);
    graph_planner_.ReEvaluateGoalPosition(goal_ptr, !master_params_.is_multi_layer);

    // Adding goal into v-graph
    FARUtil::Timer.start_time("Adding Goal to V-Graph");
    graph_planner_.UpdateGoalNavNodeConnects(goal_ptr); 
    graph_planner_.UpdaetVGraph(graph_manager_.GetNavGraph());
    if (!FARUtil::IsDebug) printf("\033[2K");
    FARUtil::Timer.end_time("Adding Goal to V-Graph");

    // Update v-graph traversibility 
    FARUtil::Timer.start_time("Path Search");
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, goal_ptr);

    // Construct path to gaol and publish waypoint
    NodePtrStack global_path;
    Point3D current_free_goal;
    NavNodePtr last_nav_ptr = nav_node_ptr_;
    bool is_planning_fails = false;
    goal_waypoint_stamped_.header.stamp = nh_->now();
    bool is_current_free_nav = false;
    bool is_reach_goal = false;
    if (graph_planner_.PathToGoal(goal_ptr, global_path, nav_node_ptr_, current_free_goal, is_planning_fails, is_reach_goal, is_current_free_nav) && nav_node_ptr_ != NULL) {
      Point3D waypoint = nav_node_ptr_->position;
      if (nav_node_ptr_ != goal_ptr) {
        waypoint = this->ProjectNavWaypoint(nav_node_ptr_, last_nav_ptr);
      } else if (master_params_.is_viewpoint_extend) {
        planner_viz_.VizViewpointExtend(goal_ptr, goal_ptr->position);
      }
      goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(waypoint);
      goal_pub_->publish(goal_waypoint_stamped_);
      is_planner_running_ = true;
      planner_viz_.VizPoint3D(waypoint, "waypoint", VizColor::MAGNA, 1.5);
      planner_viz_.VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
      planner_viz_.VizPath(global_path, is_current_free_nav);
    } else if (is_planner_running_) { // stop robot
      global_path.clear();
      planner_viz_.VizPath(global_path);
      is_planner_running_ = false;
      nav_heading_ = Point3D(0,0,0);
      if (is_planning_fails) { // stops the robot
        goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
        goal_pub_->publish(goal_waypoint_stamped_);
      }
    }
    if (!FARUtil::IsDebug) printf("\033[2K");

    // publish planner status and timers
    auto reach_goal_msg = std_msgs::msg::Bool();
    reach_goal_msg.data = is_reach_goal;
    reach_goal_pub_->publish(reach_goal_msg);
    auto traverse_timer = std_msgs::msg::Float32();
    traverse_timer.data = FARUtil::Timer.record_time("Overall_executing");
    traverse_time_pub_->publish(traverse_timer);
    if (is_reach_goal) {
      FARUtil::Timer.end_time("Overall_executing", false);
    }

    plan_timer_.data = FARUtil::Timer.end_time("Path Search");
    planning_time_pub_->publish(plan_timer_);
  }
}

void FARMaster::LocalBoundaryHandler(const std::vector<PointPair>& local_boundary) {
  if (!master_params_.is_pub_boundary || local_boundary.empty()) return;

  geometry_msgs::msg::PolygonStamped boundary_poly;
  boundary_poly.header.frame_id = master_params_.world_frame;
  boundary_poly.header.stamp = nh_->now(); // Using the ROS2 method to get the current time

  float index_z = robot_pos_.z;
  std::vector<PointPair> sorted_boundary;
  for (const auto& edge : local_boundary) {
    if (FARUtil::DistanceToLineSeg2D(robot_pos_, edge) > master_params_.local_planner_range) continue;
    sorted_boundary.push_back(edge);
  }
  FARUtil::SortEdgesClockWise(robot_pos_, sorted_boundary); /* For better rviz visualization purpose only! */
  for (const auto& edge : sorted_boundary) {
    geometry_msgs::msg::Point32 geo_p1, geo_p2;
    geo_p1.x = edge.first.x,  geo_p1.y = edge.first.y,  geo_p1.z = index_z;
    geo_p2.x = edge.second.x, geo_p2.y = edge.second.y, geo_p2.z = index_z;
    boundary_poly.polygon.points.push_back(geo_p1), boundary_poly.polygon.points.push_back(geo_p2);
    index_z += 0.001f; // separate polygon lines
  }
  boundary_pub_->publish(boundary_poly);
}


Point3D FARMaster::ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr) {
  bool is_momentum = false;
  if (last_point_ptr == nav_node_ptr || (last_point_ptr != NULL && (last_point_ptr->position - nav_node_ptr_->position).norm() < FARUtil::kNearDist)) {
    is_momentum = true;
  }
  Point3D waypoint = nav_node_ptr->position;
  float free_dist = master_params_.local_planner_range;
  const Point3D extend_p = this->ExtendViewpointOnObsCloud(nav_node_ptr_, FARUtil::surround_obs_cloud_, free_dist);
  free_dist = std::max(free_dist, master_params_.robot_dim * 2.5f);
  if (master_params_.is_viewpoint_extend) {
    waypoint = extend_p;
    planner_viz_.VizViewpointExtend(nav_node_ptr_, waypoint);
  }
  const Point3D diff_p = waypoint - robot_pos_;
  Point3D new_heading;
  if (is_momentum && nav_heading_.norm() > FARUtil::kEpsilon) {
    const float hdist = free_dist / 2.0f;
    const float ratio = std::min(hdist, diff_p.norm()) / hdist;
    new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0f - ratio);
  } else {
    new_heading = diff_p.normalize();
  }
  if (nav_heading_.norm() > FARUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0f) { // negative direction reproject
    Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
    if (temp_heading.norm_dot(new_heading) < 0.0f) {
      temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
    }
    new_heading = temp_heading;
  }
  nav_heading_ = new_heading.normalize();
  if (diff_p.norm() < free_dist) {
    waypoint = waypoint + nav_heading_ * (free_dist - diff_p.norm());
  }
  return waypoint;
}

Point3D FARMaster::ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist) {
  if (nav_node_ptr->free_direct != NodeFreeDirect::CONVEX || obsCloudIn->empty()) return nav_node_ptr->position;
  FARUtil::CropPCLCloud(obsCloudIn, viewpoint_around_ptr_, nav_node_ptr->position, free_dist + FARUtil::kNearDist);
  float maxR = std::min((nav_node_ptr->position - robot_pos_).norm(), free_dist) - FARUtil::kNearDist;
  maxR = std::max(maxR, 0.0f);
  bool is_wall = false;
  const Point3D direct = -FARUtil::SurfTopoDirect(nav_node_ptr->surf_dirs, is_wall);
  if (!is_wall) {
    Point3D waypoint = nav_node_ptr->position;
    if (viewpoint_around_ptr_->empty()) {
      waypoint = waypoint + direct * maxR;
    } else {
      kdtree_viewpoint_obs_cloud_->setInputCloud(viewpoint_around_ptr_);
      const int N_Thred = (int)std::floor(FARUtil::kNearDist / FARUtil::kLeafSize);
      const float R = FARUtil::kNearDist / 2.0f + FARUtil::kLeafSize;
      // ray tracing
      Point3D start_p = waypoint + direct * FARUtil::kNearDist;
      float ray_dist  = FARUtil::kNearDist; 
      bool is_occupied = int(FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_)) > N_Thred;
      waypoint = start_p;
      while (!is_occupied && ray_dist < free_dist) {
        start_p = start_p + direct * FARUtil::kNearDist;
        ray_dist += FARUtil::kNearDist;
        is_occupied = int(FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_)) > N_Thred;
        if (ray_dist < maxR) {
          waypoint = start_p;
        }
      }
      if (is_occupied) {
        waypoint = (nav_node_ptr->position + waypoint - direct * FARUtil::kNearDist) / 2.0f;
        waypoint.z = nav_node_ptr->position.z;
        free_dist = ray_dist - FARUtil::kNearDist;
      }
      return waypoint;
    }
  }
  return nav_node_ptr->position;
}


void FARMaster::LoadROSParams() {
  const std::string map_prefix      = "map_handler";
  const std::string scan_prefix     = "scan_handler";
  const std::string cdetect_prefix  = "c_detector";
  const std::string graph_prefix    = "graph";
  const std::string viz_prefix      = "viz";
  const std::string utility_prefix  = "util";
  const std::string planner_prefix  = "g_planner";
  const std::string contour_prefix  = "contour_graph";
  const std::string msger_prefix    = "graph_msger";

   // master params
  nh_->declare_parameter<float>("main_run_freq", 5.0);
  nh_->declare_parameter<float>("voxel_dim", 0.2);
  nh_->declare_parameter<float>("robot_dim", 0.8);
  nh_->declare_parameter<float>("vehicle_height", 0.75);
  nh_->declare_parameter<float>("sensor_range", 30.0);
  nh_->declare_parameter<float>("terrain_range", 15.0);
  nh_->declare_parameter<float>("local_planner_range", 5.0);
  nh_->declare_parameter<float>("visualize_ratio", 1.0);
  nh_->declare_parameter<bool>("is_viewpoint_extend", true);
  nh_->declare_parameter<bool>("is_multi_layer", false);
  nh_->declare_parameter<bool>("is_opencv_visual", true);
  nh_->declare_parameter<bool>("is_static_env", true);
  nh_->declare_parameter<bool>("is_pub_boundary", true);
  nh_->declare_parameter<bool>("is_debug_output", false);
  nh_->declare_parameter<bool>("is_attempt_autoswitch", true);
  nh_->declare_parameter<std::string>("world_frame", "map");
  
  // Get parameters
  nh_->get_parameter("main_run_freq", master_params_.main_run_freq);
  nh_->get_parameter("voxel_dim", master_params_.voxel_dim);
  nh_->get_parameter("robot_dim", master_params_.robot_dim);
  nh_->get_parameter("vehicle_height", master_params_.vehicle_height);
  nh_->get_parameter("sensor_range", master_params_.sensor_range);
  nh_->get_parameter("terrain_range", master_params_.terrain_range);
  nh_->get_parameter("local_planner_range", master_params_.local_planner_range);
  nh_->get_parameter("visualize_ratio", master_params_.viz_ratio);
  nh_->get_parameter("is_viewpoint_extend", master_params_.is_viewpoint_extend);
  nh_->get_parameter("is_multi_layer", master_params_.is_multi_layer);
  nh_->get_parameter("is_opencv_visual", master_params_.is_visual_opencv);
  nh_->get_parameter("is_static_env", master_params_.is_static_env);
  nh_->get_parameter("is_pub_boundary", master_params_.is_pub_boundary);
  nh_->get_parameter("is_debug_output", master_params_.is_debug_output);
  nh_->get_parameter("is_attempt_autoswitch", master_params_.is_attempt_autoswitch);
  nh_->get_parameter<std::string>("world_frame", master_params_.world_frame);
  master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);

  // print voxel_dim paramter in ROS2
  RCLCPP_INFO(nh_->get_logger(), "voxel_dim: %f", master_params_.voxel_dim);

  // Declare map parameters
  nh_->declare_parameter<float>(map_prefix + "/floor_height", 2.0);
  nh_->declare_parameter<float>(map_prefix + "/cell_length", 5.0);
  nh_->declare_parameter<float>(map_prefix + "/map_grid_max_length", 1000.0);
  nh_->declare_parameter<float>(map_prefix + "/map_grad_max_height", 100.0);

  // Get map parameters
  nh_->get_parameter(map_prefix + "/floor_height", map_params_.floor_height);
  nh_->get_parameter(map_prefix + "/cell_length", map_params_.cell_length);
  nh_->get_parameter(map_prefix + "/map_grid_max_length", map_params_.grid_max_length);
  nh_->get_parameter(map_prefix + "/map_grad_max_height", map_params_.grid_max_height);

  // Compute dependent parameters
  map_params_.height_voxel_dim = master_params_.voxel_dim * 2.0f;
  map_params_.cell_height = map_params_.floor_height / 2.5f;
  map_params_.sensor_range = master_params_.sensor_range;

  // Declare utility parameters
  nh_->declare_parameter<float>(utility_prefix + "/angle_noise", 15.0);
  nh_->declare_parameter<float>(utility_prefix + "/accept_max_align_angle", 15.0);
  nh_->declare_parameter<float>(utility_prefix + "/new_intensity_thred", 2.0);
  nh_->declare_parameter<float>(utility_prefix + "/nav_clear_dist", 0.5);
  nh_->declare_parameter<float>(utility_prefix + "/terrain_free_Z", 0.1);
  nh_->declare_parameter<int>(utility_prefix   + "/dyosb_update_thred", 4);
  nh_->declare_parameter<int>(utility_prefix   + "/new_point_counter", 10);
  nh_->declare_parameter<float>(utility_prefix + "/dynamic_obs_dacay_time", 10.0);
  nh_->declare_parameter<float>(utility_prefix + "/new_points_decay_time", 2.0);
  nh_->declare_parameter<int>(utility_prefix   + "/obs_inflate_size", 2);

  // Get utility parameters
  nh_->get_parameter(utility_prefix + "/angle_noise", FARUtil::kAngleNoise);
  nh_->get_parameter(utility_prefix + "/accept_max_align_angle", FARUtil::kAcceptAlign);
  nh_->get_parameter(utility_prefix + "/new_intensity_thred", FARUtil::kNewPIThred);
  nh_->get_parameter(utility_prefix + "/nav_clear_dist", FARUtil::kNavClearDist);
  nh_->get_parameter(utility_prefix + "/terrain_free_Z", FARUtil::kFreeZ);
  nh_->get_parameter(utility_prefix + "/dyosb_update_thred", FARUtil::kDyObsThred);
  nh_->get_parameter(utility_prefix + "/new_point_counter", FARUtil::KNewPointC);
  nh_->get_parameter(utility_prefix + "/dynamic_obs_dacay_time", FARUtil::kObsDecayTime);
  nh_->get_parameter(utility_prefix + "/new_points_decay_time", FARUtil::kNewDecayTime);
  nh_->get_parameter(utility_prefix + "/obs_inflate_size", FARUtil::kObsInflate);
  FARUtil::kLeafSize       = master_params_.voxel_dim;
  FARUtil::kNearDist       = master_params_.robot_dim;
  FARUtil::kHeightVoxel    = map_params_.height_voxel_dim;
  FARUtil::kMatchDist      = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize;
  FARUtil::kNavClearDist   = master_params_.robot_dim / 2.0f + FARUtil::kLeafSize;
  FARUtil::kProjectDist    = master_params_.voxel_dim;
  FARUtil::worldFrameId    = master_params_.world_frame;
  FARUtil::kVizRatio       = master_params_.viz_ratio;
  FARUtil::kTolerZ         = map_params_.floor_height - FARUtil::kHeightVoxel;
  FARUtil::kCellLength     = map_params_.cell_length;
  FARUtil::kCellHeight     = map_params_.cell_height;
  FARUtil::kAcceptAlign    = FARUtil::kAcceptAlign / 180.0f * M_PI;
  FARUtil::kAngleNoise     = FARUtil::kAngleNoise  / 180.0f * M_PI; 
  FARUtil::robot_dim       = master_params_.robot_dim;
  FARUtil::IsStaticEnv     = master_params_.is_static_env;
  FARUtil::IsDebug         = master_params_.is_debug_output;
  FARUtil::IsMultiLayer    = master_params_.is_multi_layer;
  FARUtil::vehicle_height  = master_params_.vehicle_height;
  FARUtil::kSensorRange    = master_params_.sensor_range;
  FARUtil::kMarginDist     = master_params_.sensor_range - FARUtil::kMatchDist;
  FARUtil::kMarginHeight   = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
  FARUtil::kTerrainRange   = master_params_.terrain_range;
  FARUtil::kLocalPlanRange = master_params_.local_planner_range;

  // graph planner params
  nh_->declare_parameter<float>(planner_prefix + "/converge_distance", 1.0);
  nh_->declare_parameter<float>(planner_prefix + "/goal_adjust_radius", 10.0);
  nh_->declare_parameter<int>(planner_prefix   + "/free_counter_thred", 5);
  nh_->declare_parameter<int>(planner_prefix   + "/reach_goal_vote_size", 5);
  nh_->declare_parameter<int>(planner_prefix   + "/path_momentum_thred", 5);
  
  nh_->get_parameter(planner_prefix + "/converge_distance", gp_params_.converge_dist);
  nh_->get_parameter(planner_prefix + "/goal_adjust_radius", gp_params_.adjust_radius);
  nh_->get_parameter(planner_prefix + "/free_counter_thred", gp_params_.free_thred);
  nh_->get_parameter(planner_prefix + "/reach_goal_vote_size", gp_params_.votes_size);
  nh_->get_parameter(planner_prefix + "/path_momentum_thred", gp_params_.momentum_thred);

  gp_params_.momentum_dist = master_params_.robot_dim / 2.0f;
  gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

  // contour graph params
  cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;

  // dynamic graph params
  nh_->declare_parameter<int>(graph_prefix   + "/connect_votes_size", 10);
  nh_->declare_parameter<int>(graph_prefix   + "/clear_dumper_thred", 3);
  nh_->declare_parameter<int>(graph_prefix   + "/node_finalize_thred", 3);
  nh_->declare_parameter<int>(graph_prefix   + "/filter_pool_size", 12);
  nh_->declare_parameter<float>(graph_prefix + "/connect_angle_thred", 10.0);
  nh_->declare_parameter<float>(graph_prefix + "/dirs_filter_margin", 10.0);

  nh_->get_parameter(graph_prefix + "/connect_votes_size", graph_params_.votes_size);
  nh_->get_parameter(graph_prefix + "/clear_dumper_thred", graph_params_.dumper_thred);
  nh_->get_parameter(graph_prefix + "/node_finalize_thred", graph_params_.finalize_thred);
  nh_->get_parameter(graph_prefix + "/filter_pool_size", graph_params_.pool_size);
  nh_->get_parameter(graph_prefix + "/connect_angle_thred", graph_params_.kConnectAngleThred);
  nh_->get_parameter(graph_prefix + "/dirs_filter_margin", graph_params_.filter_dirs_margin);

  graph_params_.filter_pos_margin        = FARUtil::kNavClearDist;
  graph_params_.filter_dirs_margin       = FARUtil::kAngleNoise;
  graph_params_.kConnectAngleThred       = FARUtil::kAcceptAlign;
  graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;

  // graph messager params
  nh_->declare_parameter<int>(msger_prefix + "/robot_id", 0);
  nh_->get_parameter(msger_prefix + "/robot_id", msger_parmas_.robot_id);

  msger_parmas_.frame_id    = master_params_.world_frame;
  msger_parmas_.votes_size  = graph_params_.votes_size;
  msger_parmas_.pool_size   = graph_params_.pool_size;
  msger_parmas_.dist_margin = graph_params_.filter_pos_margin;

  // scan handler params
  scan_params_.terrain_range = master_params_.terrain_range;
  scan_params_.voxel_size    = master_params_.voxel_dim;
  scan_params_.ceil_height   = map_params_.floor_height;

  // contour detector params
  nh_->declare_parameter<float>(cdetect_prefix + "/resize_ratio", 5.0);
  nh_->declare_parameter<int>(cdetect_prefix   + "/filter_count_value", 5);
  nh_->declare_parameter<bool>(cdetect_prefix  + "/is_save_img", false);
  nh_->declare_parameter<std::string>(cdetect_prefix + "/img_folder_path", "");

  nh_->get_parameter(cdetect_prefix + "/resize_ratio", cdetect_params_.kRatio);
  nh_->get_parameter(cdetect_prefix + "/filter_count_value", cdetect_params_.kThredValue);
  nh_->get_parameter(cdetect_prefix + "/is_save_img", cdetect_params_.is_save_img);
  nh_->get_parameter(cdetect_prefix + "/img_folder_path", cdetect_params_.img_path);

  cdetect_params_.kBlurSize    = (int)std::round(FARUtil::kNavClearDist / master_params_.voxel_dim);
  cdetect_params_.sensor_range = master_params_.sensor_range;
  cdetect_params_.voxel_dim    = master_params_.voxel_dim;
  
  // Visibility graph save/load params
  nh_->declare_parameter<bool>("vgraph_autoload", false);
  nh_->declare_parameter<std::string>("vgraph_file_path", "");
  nh_->declare_parameter<bool>("vgraph_autosave", false);
  nh_->declare_parameter<float>("vgraph_save_interval", 60.0);
  
  bool vgraph_autoload = false;
  std::string vgraph_file_path = "";
  
  nh_->get_parameter("vgraph_autoload", vgraph_autoload);
  nh_->get_parameter("vgraph_file_path", vgraph_file_path);
  
  RCLCPP_WARN(nh_->get_logger(), "=== VGRAPH CONFIG: autoload=%s, path='%s' ===", 
              vgraph_autoload ? "TRUE" : "FALSE", vgraph_file_path.c_str());
  
  // Auto-load visibility graph if enabled
  if (vgraph_autoload && !vgraph_file_path.empty()) {
    std::ifstream test_file(vgraph_file_path);
    if (test_file.good()) {
      test_file.close();
      RCLCPP_WARN(nh_->get_logger(), "=== VGRAPH: File found! Setting pending load flag ===");
      // Set flag for MainLoopCallBack to load the graph when odom is ready
      pending_graph_load_path_ = vgraph_file_path;
      is_pending_graph_load_ = true;
    } else {
      RCLCPP_WARN(nh_->get_logger(), "Visibility graph file not found: %s", vgraph_file_path.c_str());
      RCLCPP_INFO(nh_->get_logger(), "Will build graph from scratch");
    }
  } else {
    RCLCPP_WARN(nh_->get_logger(), "=== VGRAPH: Auto-load NOT enabled (autoload=%s, path_empty=%s) ===",
                vgraph_autoload ? "true" : "false", vgraph_file_path.empty() ? "true" : "false");
  }
}

void FARMaster::OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // transform from odom frame to mapping frame
  std::string odom_frame = msg->header.frame_id;
  tf2::Transform tf_odom_pose;
  tf2::fromMsg(msg->pose.pose, tf_odom_pose);
  
  if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) RCLCPP_WARN_ONCE(nh_->get_logger(), "FARMaster: odom frame does NOT match with world frame!");
    tf2::Transform odom_to_world_tf_stamp;
    try
    {
      tf_buffer_->canTransform(master_params_.world_frame, odom_frame, tf2::TimePointZero, std::chrono::seconds(1));
      auto transform_stamped = tf_buffer_->lookupTransform(master_params_.world_frame, odom_frame, tf2::TimePointZero);
      tf2::fromMsg(transform_stamped.transform, odom_to_world_tf_stamp);
      tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
    }
    catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(nh_->get_logger(), "Tracking odom TF lookup: %s",ex.what());
      return;
    }
  }
  
  robot_pos_.x = tf_odom_pose.getOrigin().x(); 
  robot_pos_.y = tf_odom_pose.getOrigin().y();
  robot_pos_.z = tf_odom_pose.getOrigin().z();

  // extract robot heading
  FARUtil::robot_pos = robot_pos_;
  double roll, pitch, yaw;
  tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
  robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

  if (!is_odom_init_) {
    // system start time
    FARUtil::systemStartTime = nh_->now().seconds();
    FARUtil::map_origin = robot_pos_;
    map_handler_.UpdateRobotPosition(robot_pos_);
  }

  is_odom_init_ = true;
}


void FARMaster::PrcocessCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc, const PointCloudPtr& cloudOut) 
{
  pcl::PointCloud<PCLPoint> temp_cloud;
  pcl::fromROSMsg(*pc, temp_cloud);
  cloudOut->clear(), *cloudOut = temp_cloud;
  if (cloudOut->empty()) return;
  FARUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
  // transform cloud frame
  std::string cloud_frame = pc->header.frame_id;
  FARUtil::RemoveNanInfPoints(cloudOut);
  if (!FARUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) RCLCPP_WARN_ONCE(nh_->get_logger(),"FARMaster: cloud frame does NOT match with world frame!");
    try
    {
      FARUtil::TransformPCLFrame(cloud_frame, 
                                 master_params_.world_frame, 
                                 tf_buffer_,
                                 cloudOut);
    }
    catch(const tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "Tracking cloud TF lookup: " << ex.what());
      return;
    }
  }
}


void FARMaster::ScanCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr scan_pc) {
  if (master_params_.is_static_env || !is_odom_init_) return;
  this->PrcocessCloud(scan_pc, FARUtil::cur_scan_cloud_);
  scan_handler_.UpdateRobotPosition(robot_pos_);
}

void FARMaster::TerrainLocalCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pc) {
  if (master_params_.is_static_env) return;
  this->PrcocessCloud(pc, local_terrain_ptr_);
  FARUtil::ExtractFreeAndObsCloud(local_terrain_ptr_, FARUtil::local_terrain_free_, FARUtil::local_terrain_obs_);
}

void FARMaster::TerrainCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pc) {
  if (!is_odom_init_) return;
  // update map grid robot center
  map_handler_.UpdateRobotPosition(FARUtil::robot_pos);
  if (!is_stop_update_) {
    this->PrcocessCloud(pc, temp_cloud_ptr_);
    FARUtil::CropBoxCloud(temp_cloud_ptr_, robot_pos_, Point3D(master_params_.terrain_range,
                                                               master_params_.terrain_range,
                                                               FARUtil::kTolerZ));
    FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
    if (!master_params_.is_static_env) {
      FARUtil::RemoveOverlapCloud(temp_obs_ptr_, FARUtil::stack_dyobs_cloud_, true);
    }
    map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
    map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
    // extract new points
    FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_,
                                     FARUtil::surround_obs_cloud_,
                                     FARUtil::cur_new_cloud_);
  } else { // stop env update
    temp_cloud_ptr_->clear();
    FARUtil::cur_new_cloud_->clear();
  }
  // extract surround free cloud & update terrain height
  map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);
  map_handler_.UpdateTerrainHeightGrid(FARUtil::surround_free_cloud_, terrain_height_ptr_);
  // update surround obs cloud
  map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
  // extract dynamic obstacles
  FARUtil::cur_dyobs_cloud_->clear();
  if (!master_params_.is_static_env && !is_stop_update_) {
    this->ExtractDynamicObsFromScan(FARUtil::cur_scan_cloud_, 
                                    FARUtil::surround_obs_cloud_, 
                                    FARUtil::surround_free_cloud_, 
                                    FARUtil::cur_dyobs_cloud_);
    if (int(FARUtil::cur_dyobs_cloud_->size()) > FARUtil::kDyObsThred) {
      if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "FARMaster: dynamic obstacle detected, size: %ld", FARUtil::cur_dyobs_cloud_->size());
      FARUtil::InflateCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true);
      map_handler_.RemoveObsCloudFromGrid(FARUtil::cur_dyobs_cloud_);
      FARUtil::RemoveOverlapCloud(FARUtil::surround_obs_cloud_, FARUtil::cur_dyobs_cloud_);
      FARUtil::FilterCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim);
      // update new cloud
      *FARUtil::cur_new_cloud_ += *FARUtil::cur_dyobs_cloud_;
      FARUtil::FilterCloud(FARUtil::cur_new_cloud_, master_params_.voxel_dim);
    }
    // update world dynamic obstacles
    FARUtil::StackCloudByTime(FARUtil::cur_dyobs_cloud_, FARUtil::stack_dyobs_cloud_, FARUtil::kObsDecayTime, nh_);
  }
  
  // create and update kdtrees
  FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime, nh_);
  FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);

  if (!FARUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;

  /* visualize clouds */
  planner_viz_.VizPointCloud(new_PCL_pub_, FARUtil::stack_new_cloud_);
  planner_viz_.VizPointCloud(dynamic_obs_pub_, FARUtil::cur_dyobs_cloud_);
  planner_viz_.VizPointCloud(surround_free_debug_, FARUtil::surround_free_cloud_);
  planner_viz_.VizPointCloud(surround_obs_debug_,  FARUtil::surround_obs_cloud_);
  planner_viz_.VizPointCloud(terrain_height_pub_, terrain_height_ptr_);
  // visualize map grid
  PointStack neighbor_centers, occupancy_centers;
  map_handler_.GetNeighborCeilsCenters(neighbor_centers);
  map_handler_.GetOccupancyCeilsCenters(occupancy_centers);
  planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.cell_length, map_params_.cell_height);
  // DBBUG visual raycast grids
  if (!master_params_.is_static_env) {
    scan_handler_.GridVisualCloud(scan_grid_ptr_, GridStatus::RAY);
    planner_viz_.VizPointCloud(scan_grid_debug_, scan_grid_ptr_);
  }
}

void FARMaster::ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                          const PointCloudPtr& obsCloudIn,
                                          const PointCloudPtr& freeCloudIn,
                                          const PointCloudPtr& dyObsCloudOut)
{
  scan_handler_.ReInitGrids();
  scan_handler_.SetCurrentScanCloud(scanCloudIn, freeCloudIn);
  scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

void FARMaster::WaypointCallBack(const geometry_msgs::msg::PointStamped& route_goal) {
  if (!is_graph_init_) {
    if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(),"FARMaster: wait for v-graph to init before sending any goals");
    return;
  }
  Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
  const std::string goal_frame = route_goal.header.frame_id;
  if (!FARUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) RCLCPP_WARN_ONCE(nh_->get_logger(), "FARMaster: waypoint published is not on world frame!");
    FARUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_buffer_, goal_p); 
  }
  graph_planner_.UpdateGoal(goal_p);
  FARUtil::Timer.start_time("Overall_executing", true);
  // visualize original goal
  planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
}

/* allocate static utility PointCloud pointer memory */
PointCloudPtr  FARUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_new_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_obs_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_free_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float FARUtil::kEpsilon = 1e-7;
const float FARUtil::kINF     = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float   FARUtil::kAngleNoise; 
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float   FARUtil::robot_dim;
float   FARUtil::vehicle_height;
float   FARUtil::kLeafSize;
float   FARUtil::kHeightVoxel;
float   FARUtil::kNavClearDist;
float   FARUtil::kCellLength;
float   FARUtil::kCellHeight;
float   FARUtil::kNewPIThred;
float   FARUtil::kSensorRange;
float   FARUtil::kMarginDist;
float   FARUtil::kMarginHeight;
float   FARUtil::kTerrainRange;
float   FARUtil::kLocalPlanRange;
float   FARUtil::kFreeZ;
float   FARUtil::kVizRatio;
double  FARUtil::systemStartTime;
float   FARUtil::kObsDecayTime;
float   FARUtil::kNewDecayTime;
float   FARUtil::kNearDist;
float   FARUtil::kMatchDist;
float   FARUtil::kProjectDist;
int     FARUtil::kDyObsThred;
int     FARUtil::KNewPointC;
int     FARUtil::kObsInflate;
float   FARUtil::kTolerZ;
float   FARUtil::kAcceptAlign;
bool    FARUtil::IsStaticEnv;
bool    FARUtil::IsDebug;
bool    FARUtil::IsMultiLayer;
TimeMeasure FARUtil::Timer;

/* Global Graph */
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t  DynamicGraph::id_tracker_;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;
std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_;

/* init static contour graph values */
CTNodeStack ContourGraph::polys_ctnodes_;
CTNodeStack ContourGraph::contour_graph_;
PolygonStack ContourGraph::contour_polygons_;
std::vector<PointPair> ContourGraph::global_contour_;
std::vector<PointPair> ContourGraph::unmatched_contour_;
std::vector<PointPair> ContourGraph::inactive_contour_;
std::vector<PointPair> ContourGraph::boundary_contour_;
std::vector<PointPair> ContourGraph::local_boundary_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_;

/* init terrain map values */
PointKdTreePtr MapHandler::kdtree_terrain_clould_;
std::vector<int> MapHandler::terrain_grid_occupy_list_;
std::vector<int> MapHandler::terrain_grid_traverse_list_;
std::unordered_set<int> MapHandler::neighbor_obs_indices_;
std::unordered_set<int> MapHandler::extend_obs_indices_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_;

/***************************************************************************************/
// Save/Load Visibility Graph Functions
/***************************************************************************************/

void FARMaster::SaveVisibilityGraph(const std::string& filename) {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs.is_open()) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to open file for saving: %s", filename.c_str());
        return;
    }

    try {
        // Get the global graph
        const NodePtrStack& graph = graph_manager_.GetNavGraph();
        
        // Save graph size
        std::size_t graph_size = graph.size();
        ofs.write(reinterpret_cast<const char*>(&graph_size), sizeof(graph_size));
        
        RCLCPP_INFO(nh_->get_logger(), "Saving %zu nodes...", graph_size);
        
        // Create ID mapping
        std::unordered_map<NavNodePtr, std::size_t> node_to_idx;
        for (std::size_t i = 0; i < graph.size(); i++) {
            node_to_idx[graph[i]] = i;
        }
        
        // Save each node
        for (const auto& node_ptr : graph) {
            if (node_ptr == NULL) continue;
            
            // Save node ID
            ofs.write(reinterpret_cast<const char*>(&node_ptr->id), sizeof(node_ptr->id));
            
            // Save position
            ofs.write(reinterpret_cast<const char*>(&node_ptr->position.x), sizeof(float));
            ofs.write(reinterpret_cast<const char*>(&node_ptr->position.y), sizeof(float));
            ofs.write(reinterpret_cast<const char*>(&node_ptr->position.z), sizeof(float));
            
            // Save node properties
            ofs.write(reinterpret_cast<const char*>(&node_ptr->is_covered), sizeof(bool));
            ofs.write(reinterpret_cast<const char*>(&node_ptr->is_frontier), sizeof(bool));
            ofs.write(reinterpret_cast<const char*>(&node_ptr->is_navpoint), sizeof(bool));
            ofs.write(reinterpret_cast<const char*>(&node_ptr->is_boundary), sizeof(bool));
            ofs.write(reinterpret_cast<const char*>(&node_ptr->free_direct), sizeof(int));
            
            // Save connections
            std::size_t connect_size = node_ptr->connect_nodes.size();
            ofs.write(reinterpret_cast<const char*>(&connect_size), sizeof(connect_size));
            for (const auto& connected_node : node_ptr->connect_nodes) {
                std::size_t idx = node_to_idx[connected_node];
                ofs.write(reinterpret_cast<const char*>(&idx), sizeof(idx));
            }
            
            // Save polygon connections
            std::size_t poly_size = node_ptr->poly_connects.size();
            ofs.write(reinterpret_cast<const char*>(&poly_size), sizeof(poly_size));
            for (const auto& poly_node : node_ptr->poly_connects) {
                std::size_t idx = node_to_idx[poly_node];
                ofs.write(reinterpret_cast<const char*>(&idx), sizeof(idx));
            }
            
            // Save contour connections
            std::size_t contour_size = node_ptr->contour_connects.size();
            ofs.write(reinterpret_cast<const char*>(&contour_size), sizeof(contour_size));
            for (const auto& contour_node : node_ptr->contour_connects) {
                std::size_t idx = node_to_idx[contour_node];
                ofs.write(reinterpret_cast<const char*>(&idx), sizeof(idx));
            }
        }
        
        ofs.close();
        RCLCPP_INFO(nh_->get_logger(), "Successfully saved visibility graph to: %s", filename.c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(nh_->get_logger(), "Error saving visibility graph: %s", e.what());
    }
}

void FARMaster::LoadVisibilityGraph(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to open file for loading: %s", filename.c_str());
        return;
    }

    try {
        // Clear current graph
        RCLCPP_INFO(nh_->get_logger(), "Clearing current visibility graph...");
        graph_manager_.ResetCurrentGraph();
        
        // Read graph size
        std::size_t graph_size;
        ifs.read(reinterpret_cast<char*>(&graph_size), sizeof(graph_size));
        
        RCLCPP_INFO(nh_->get_logger(), "Loading %zu nodes...", graph_size);
        
        std::size_t max_id = 0;
        
        // Create nodes first with FULL initialization (mimicking CreateNavNodeFromPoint)
        std::vector<NavNodePtr> loaded_nodes(graph_size);
        for (std::size_t i = 0; i < graph_size; i++) {
            NavNodePtr node_ptr = std::make_shared<NavNode>();
            
            // Read node ID
            ifs.read(reinterpret_cast<char*>(&node_ptr->id), sizeof(node_ptr->id));
            if (node_ptr->id > max_id) max_id = node_ptr->id;
            
            // Read position
            float x, y, z;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&y), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&z), sizeof(float));
            node_ptr->position = Point3D(x, y, z);
            
            // Read saved node properties
            ifs.read(reinterpret_cast<char*>(&node_ptr->is_covered), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&node_ptr->is_frontier), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&node_ptr->is_navpoint), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&node_ptr->is_boundary), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&node_ptr->free_direct), sizeof(int));
            
            // Initialize ALL other node properties properly (mimic CreateNavNodeFromPoint)
            node_ptr->pos_filter_vec.clear();
            node_ptr->pos_filter_vec.push_back(node_ptr->position);
            node_ptr->surf_dirs_vec.clear();
            node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
            node_ptr->ctnode = NULL;
            node_ptr->is_active = true;
            node_ptr->is_block_frontier = false;
            node_ptr->is_contour_match = false;
            node_ptr->is_odom = false;
            node_ptr->is_near_nodes = true;
            node_ptr->is_wide_near = true;
            node_ptr->is_merged = false;
            node_ptr->is_finalized = true;  // Mark as finalized so dynamic graph doesn't try to refine
            node_ptr->is_traversable = true;
            node_ptr->is_free_traversable = true;
            node_ptr->is_goal = false;
            node_ptr->clear_dumper_count = 0;
            node_ptr->frontier_votes.clear();
            node_ptr->invalid_boundary.clear();
            node_ptr->connect_nodes.clear();
            node_ptr->poly_connects.clear();
            node_ptr->contour_connects.clear();
            node_ptr->contour_votes.clear();
            node_ptr->potential_contours.clear();
            node_ptr->trajectory_connects.clear();
            node_ptr->trajectory_votes.clear();
            node_ptr->terrain_votes.clear();
            node_ptr->potential_edges.clear();
            node_ptr->edge_votes.clear();
            // Planner members
            node_ptr->is_block_to_goal = false;
            node_ptr->gscore = FARUtil::kINF;
            node_ptr->fgscore = FARUtil::kINF;
            node_ptr->parent = NULL;
            node_ptr->free_parent = NULL;
            
            // Skip connection data for now (read but don't use yet)
            std::size_t connect_size;
            ifs.read(reinterpret_cast<char*>(&connect_size), sizeof(connect_size));
            for (std::size_t j = 0; j < connect_size; j++) {
                std::size_t idx;
                ifs.read(reinterpret_cast<char*>(&idx), sizeof(idx));
            }
            
            std::size_t poly_size;
            ifs.read(reinterpret_cast<char*>(&poly_size), sizeof(poly_size));
            for (std::size_t j = 0; j < poly_size; j++) {
                std::size_t idx;
                ifs.read(reinterpret_cast<char*>(&idx), sizeof(idx));
            }
            
            std::size_t contour_size;
            ifs.read(reinterpret_cast<char*>(&contour_size), sizeof(contour_size));
            for (std::size_t j = 0; j < contour_size; j++) {
                std::size_t idx;
                ifs.read(reinterpret_cast<char*>(&idx), sizeof(idx));
            }
            
            loaded_nodes[i] = node_ptr;
            // Add to global graph AND register in idx_node_map
            DynamicGraph::AddNodeToGraph(node_ptr);
            DynamicGraph::RegisterNodeWithId(node_ptr);
        }
        
        // Set id_tracker_ to be beyond all loaded IDs to avoid conflicts
        DynamicGraph::SetIdTracker(max_id + 1);
        
        // Now re-read file to restore connections
        ifs.clear();
        ifs.seekg(sizeof(graph_size), std::ios::beg);
        
        for (std::size_t i = 0; i < graph_size; i++) {
            NavNodePtr node_ptr = loaded_nodes[i];
            
            // Skip to connection data (re-read node fields)
            std::size_t node_id;
            float x, y, z;
            bool b1, b2, b3, b4;
            int fd;
            
            ifs.read(reinterpret_cast<char*>(&node_id), sizeof(node_id));
            ifs.read(reinterpret_cast<char*>(&x), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&y), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&z), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&b1), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&b2), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&b3), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&b4), sizeof(bool));
            ifs.read(reinterpret_cast<char*>(&fd), sizeof(int));
            
            // Read and restore connections
            std::size_t connect_size;
            ifs.read(reinterpret_cast<char*>(&connect_size), sizeof(connect_size));
            for (std::size_t j = 0; j < connect_size; j++) {
                std::size_t idx;
                ifs.read(reinterpret_cast<char*>(&idx), sizeof(idx));
                if (idx < loaded_nodes.size()) {
                    DynamicGraph::AddEdge(node_ptr, loaded_nodes[idx]);
                }
            }
            
            std::size_t poly_size;
            ifs.read(reinterpret_cast<char*>(&poly_size), sizeof(poly_size));
            for (std::size_t j = 0; j < poly_size; j++) {
                std::size_t idx;
                ifs.read(reinterpret_cast<char*>(&idx), sizeof(idx));
                if (idx < loaded_nodes.size()) {
                    DynamicGraph::AddPolyEdge(node_ptr, loaded_nodes[idx]);
                }
            }
            
            std::size_t contour_size;
            ifs.read(reinterpret_cast<char*>(&contour_size), sizeof(contour_size));
            for (std::size_t j = 0; j < contour_size; j++) {
                std::size_t idx;
                ifs.read(reinterpret_cast<char*>(&idx), sizeof(idx));
                if (idx < loaded_nodes.size()) {
                    // Restore contour connections
                    if (!FARUtil::IsTypeInStack(loaded_nodes[idx], node_ptr->contour_connects)) {
                        node_ptr->contour_connects.push_back(loaded_nodes[idx]);
                    }
                }
            }
        }
        
        ifs.close();
        
  // The loaded graph is now integrated, but we intentionally request a refresh cycle
  // so dynamic updates (edges validity, near-node sets, etc.) can kick in immediately
  // when the RViz "Update Visibility Graph" checkbox is enabled.
  //
  // NOTE: Do NOT set is_graph_init_=true here. It will be set by MainLoopCallBack
  // once the next update cycle runs.
  is_graph_init_ = false;
        RCLCPP_INFO(nh_->get_logger(), "Successfully loaded visibility graph from: %s", filename.c_str());
        RCLCPP_INFO(nh_->get_logger(), "Loaded %zu nodes with connections (id_tracker set to %zu)", graph_size, max_id + 1);
        
        // Update nav_graph_ with loaded graph
        nav_graph_ = graph_manager_.GetNavGraph();
        RCLCPP_INFO(nh_->get_logger(), "Updating visualization and contour extraction...");
        
        // Update all dependent modules with loaded graph
        contour_graph_.ExtractGlobalContours();      // Extract polygons from loaded graph
        graph_planner_.UpdaetVGraph(nav_graph_);     // Update graph planner
        graph_msger_.UpdateGlobalGraph(nav_graph_);  // Update graph messager
        
        // Visualize the loaded graph immediately
        planner_viz_.VizGraph(nav_graph_);
        planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);
        
        RCLCPP_INFO(nh_->get_logger(), "Graph integration complete. System ready for dynamic updates.");

  // If updates are currently enabled, force a rebuild/update pass on the next tick.
  // If updates are frozen, this flag will still be honored once the user resumes.
  // (MainLoopCallBack will set is_graph_init_=true after that pass.)
  // Nothing else needed here.
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(nh_->get_logger(), "Error loading visibility graph: %s", e.what());
    }
}


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  
  auto far_planner_node = std::make_shared<FARMaster>();
  far_planner_node->Init();
  rclcpp::spin(far_planner_node->GetNodeHandle());

  rclcpp::shutdown();

  return 0;
}