/*
 * CUDA Wrapper for FAR Planner GPU Acceleration
 * Copyright (C) 2025 - GPU Acceleration Implementation
 */

#ifndef FAR_PLANNER_CUDA_WRAPPER_H
#define FAR_PLANNER_CUDA_WRAPPER_H

#include "far_planner/cuda_types.h"
#include "far_planner/node_struct.h"
#include "far_planner/point_struct.h"
#include <vector>
#include <memory>

// External CUDA kernel functions
extern "C" {
    void LaunchVisibilityCheckKernel(const NavNode_GPU* d_nodes,
                                     int* d_connection_matrix,
                                     const PolygonEdge_GPU* d_polygons,
                                     int num_nodes,
                                     int num_polygons,
                                     float project_dist,
                                     float nav_clear_dist);
    
    bool IsCudaAvailable();
    void GetGPUInfo(int* device_count, char* device_name, int name_len);
}

namespace FARPlannerCUDA {

class CUDAVisibilityChecker {
public:
    CUDAVisibilityChecker();
    ~CUDAVisibilityChecker();
    
    // Initialize CUDA
    void InitCUDA();
    
    // Cleanup CUDA resources
    void CleanupCUDA();
    
    // Check if CUDA is available
    static bool IsCUDAAvailable();
    
    // Get GPU information
    static std::string GetGPUInfo();
    
    // Main visibility checking function
    // Returns a 2D connection matrix where matrix[i][j] = 1 means nodes i and j can see each other
    std::vector<std::vector<int>> CheckVisibilityConnections(
        const std::vector<NavNodePtr>& nodes,
        const std::vector<PointPair>& contour_edges,
        float project_dist,
        float nav_clear_dist
    );

private:
    bool cuda_initialized_;
    
    // Helper functions to convert data structures
    NavNode_GPU ConvertToGPUNode(const NavNodePtr& node);
    PolygonEdge_GPU ConvertToGPUEdge(const PointPair& edge);
};

} // namespace FARPlannerCUDA

#endif // FAR_PLANNER_CUDA_WRAPPER_H
