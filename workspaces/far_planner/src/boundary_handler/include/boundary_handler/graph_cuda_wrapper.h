/*
 * CUDA Wrapper for Visibility Graph GPU Acceleration
 * Copyright (C) 2025 - GPU Acceleration Implementation
 */

#ifndef GRAPH_CUDA_WRAPPER_H
#define GRAPH_CUDA_WRAPPER_H

#include <vector>
#include <memory>
#include "boundary_handler/point_struct.h"
#include "boundary_handler/graph_cuda_types.h"

namespace GraphCUDA {

/**
 * @brief CUDA-accelerated visibility graph construction wrapper
 */
class CUDAGraphBuilder {
public:
    CUDAGraphBuilder();
    ~CUDAGraphBuilder();
    
    /**
     * @brief Check if CUDA is available on this system
     */
    static bool IsCUDAAvailable();
    
    /**
     * @brief Get GPU device information
     */
    static std::string GetGPUInfo();
    
    /**
     * @brief Build visibility connections using GPU acceleration
     * 
     * @param nodes Vector of navigation nodes (host memory)
     * @param polygons Vector of polygon vertices (host memory)
     * @param height_tolz Height tolerance for connections
     * @return Connection matrix (symmetric, num_nodes x num_nodes)
     */
    std::vector<std::vector<int>> BuildVisibilityGraph(
        const std::vector<NavNode_GPU>& nodes,
        const std::vector<PolygonVertex_GPU>& polygons,
        float height_tolz
    );
    
private:
    bool cuda_initialized_;
    
    void InitCUDA();
    void CleanupCUDA();
};

} // namespace GraphCUDA

// C-style interface for CUDA kernel
extern "C" {
    void LaunchVisibilityGraphKernel(const NavNode_GPU* d_nodes,
                                     int* d_connection_matrix,
                                     const PolygonVertex_GPU* d_polygons,
                                     int num_nodes,
                                     int total_vertices,
                                     float height_tolz);
    
    bool IsCudaAvailable();
    void GetGPUInfo(int* device_count, char* device_name, int name_len);
}

#endif // GRAPH_CUDA_WRAPPER_H
