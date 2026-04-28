# FAR planner ROS 2 services (what changed)

This note explains the code changes that were made to `far_planner_node` to make the ROS 2 service API work (start/stop planning + visibility-graph control + save/load/clear).

## Goals of the change

- Provide a **service-based control API** (CLI / RViz / scripts) instead of relying only on topic messages.
- Allow the visibility graph (V-Graph) to be:
  - **frozen/resumed** on demand
  - **cleared** immediately
  - **loaded/saved** via a file path parameter
- Add a clean **start/stop** interface for the FAR planning loop:
  - Planning should run only after a start request.
  - Stop should publish a “stop waypoint” (current robot position) so the robot halts.

## Files changed

### 1) `workspaces/far_planner/src/far_planner/include/far_planner/far_planner.h`

**What was added**

- Included Trigger service definition:
  - `#include <std_srvs/srv/trigger.hpp>`

- Added ROS 2 service members to `class FARMaster`:
  - `start_far_planner_srv_` and `stop_far_planner_srv_` (new)
  - `stop_vgraph_update_srv_` and `resume_vgraph_update_srv_`
  - `clear_vgraph_srv_`, `load_vgraph_srv_`, `save_vgraph_srv_`

- Added a new state flag:
  - `bool is_system_started_;`

**Why**

- The FAR planner needed an internal boolean gate for “planning enabled” vs “planning disabled”.
- The node needed to own service handles so the service servers stay alive for the node lifetime.

### 2) `workspaces/far_planner/src/far_planner/src/far_planner.cpp`

#### A) New services created in `FARMaster::Init()`

All services use `std_srvs/srv/Trigger` (empty request, boolean success + message response).

##### 1. `/start_far_planner`

- Sets:
  - `is_system_started_ = true`
- Safety check:
  - If `is_init_completed_` is false, the service returns `success=false` with a message.
- Effect:
  - Allows `PlanningCallBack()` to run (planning enabled).

##### 2. `/stop_far_planner`

- Sets:
  - `is_system_started_ = false`
- Publishes a *stop waypoint*:
  - `goal_waypoint_stamped_.point = robot_pos_`
  - Publishes it on `/way_point` via `goal_pub_`
- Clears path visualization:
  - `planner_viz_.VizPath(empty_path)`
- Resets planning-related state:
  - `is_planner_running_ = false`
  - `nav_heading_ = (0,0,0)`

##### 3. V-Graph control services

- `/stop_visibility_graph_update`
  - Sets `is_stop_update_ = true` (freezes dynamic updates)

- `/resume_visibility_graph_update`
  - Sets `is_stop_update_ = false`
  - Forces a rebuild/refresh by setting `is_graph_init_ = false`

- `/clear_visibility_graph`
  - Designed to clear immediately (not waiting for timers / preconditions):
    - Freezes updates: `is_stop_update_ = true`
    - Clears any pending load: `is_pending_graph_load_ = false`, `pending_graph_load_path_.clear()`
    - Clears the param `vgraph_file_path` back to empty string
    - Calls `ResetEnvironmentAndGraph()` immediately
    - Clears `is_reset_env_` to avoid repeated resets

- `/load_visibility_graph`
  - Reads the ROS parameter `vgraph_file_path` from the **node** (`far_planner_node`).
  - If the param is empty: returns `success=false`.
  - Otherwise calls:
    - `LoadVisibilityGraph(filename)`

- `/save_visibility_graph`
  - Same pattern as load:
    - reads `vgraph_file_path`
    - errors if empty
    - otherwise calls `SaveVisibilityGraph(filename)`

#### B) Initialization defaults

In `FARMaster::Init()` the following defaults were set:

- `is_stop_update_ = true`
  - Default behavior is **V-Graph frozen** until explicitly resumed.
  - This supports loading a saved VGH and keeping it unchanged.

- `is_system_started_ = false`
  - Default behavior is **planning disabled** until `/start_far_planner` is called.

- `is_pending_graph_load_` and `pending_graph_load_path_` are initialized early (before `LoadROSParams()`).
  - This avoids uninitialized state if ROS parameters set an auto-load path.

#### C) Planning loop gating

The planning timer still calls `FARMaster::PlanningCallBack()` at the configured frequency, but the callback now starts with:

- `if (!is_init_completed_ || !is_graph_init_ || !is_system_started_) return;`

Meaning planning runs only when:

1. init is complete (`is_init_completed_ == true`)
2. the graph is initialized (`is_graph_init_ == true`)
3. the system has been started by service (`is_system_started_ == true`)

## How to use (CLI)

### Start/Stop FAR planning

```bash
ros2 service call /start_far_planner std_srvs/srv/Trigger {}
ros2 service call /stop_far_planner  std_srvs/srv/Trigger {}
```

### Freeze / Resume V-Graph updates

```bash
ros2 service call /stop_visibility_graph_update   std_srvs/srv/Trigger {}
ros2 service call /resume_visibility_graph_update std_srvs/srv/Trigger {}
```

### Clear the visibility graph

```bash
ros2 service call /clear_visibility_graph std_srvs/srv/Trigger "{}"
```

### Load / Save visibility graph from/to a file

These services use a node parameter called `vgraph_file_path`.

1) Set a path on the node:

```bash
ros2 param set /far_planner_node vgraph_file_path /home/tharushi/Go2-Dynamic-Inspection/saved_vgraphs/vgraph.vgh
```

2) Call load or save:

```bash
ros2 service call /load_visibility_graph std_srvs/srv/Trigger "{}"
ros2 service call /save_visibility_graph std_srvs/srv/Trigger "{}"
```

### Common gotcha: node name vs package name

The node is created as:

- `rclcpp::Node::make_shared("far_planner_node")`

So parameters are attached to `/far_planner_node`.

This is why this works:

- `ros2 param set /far_planner_node vgraph_file_path ...`

…and this usually won’t (unless you have a different node with that name):

- `ros2 param set /far_planner vgraph_file_path ...`

## Quick sanity checks

- List services:
  - `ros2 service list | grep -E "far_planner|visibility_graph"`
- Inspect the parameter:
  - `ros2 param get /far_planner_node vgraph_file_path`

## Summary

- Added service API for starting/stopping planning.
- Added/finished service API for freezing/resuming/clearing/loading/saving the visibility graph.
- Added a planning gate so `PlanningCallBack()` only runs after `/start_far_planner`.
