/*
 * CUDA Kernels for GPU-Accelerated Collision Detection in FAR Planner
 * Copyright (C) 2025 - GPU Acceleration Implementation
 */

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include "far_planner/cuda_types.h"

// Device helper functions for 2D line segment intersection
__device__ bool doIntersect_GPU(const Point3D_GPU& p1, const Point3D_GPU& q1,
                                 const Point3D_GPU& p2, const Point3D_GPU& q2) {
    // Helper lambda for orientation
    auto orientation = [](const Point3D_GPU& p, const Point3D_GPU& q, const Point3D_GPU& r) -> int {
        float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        if (fabsf(val) < 1e-7f) return 0;  // Collinear
        return (val > 0) ? 1 : 2;  // Clockwise or Counterclockwise
    };
    
    auto onSegment = [](const Point3D_GPU& p, const Point3D_GPU& q, const Point3D_GPU& r) -> bool {
        return (q.x <= fmaxf(p.x, r.x) && q.x >= fminf(p.x, r.x) &&
                q.y <= fmaxf(p.y, r.y) && q.y >= fminf(p.y, r.y));
    };
    
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    
    return false;
}

__device__ bool IsTypeInStack_GPU(int elem, const int* stack, int stack_size) {
    for (int i = 0; i < stack_size; i++) {
        if (stack[i] == elem) return true;
    }
    return false;
}

__device__ bool IsOutReducedDirs_GPU(const Point3D_GPU& diff_p, 
                                     const Point3D_GPU& surf_dir_first,
                                     const Point3D_GPU& surf_dir_second) {
    Point3D_GPU norm_dir = diff_p.normalize_flat();
    Point3D_GPU temp_opposite_dir;
    
    // Check first half range
    temp_opposite_dir = surf_dir_second * (-1.0f);
    float in_res_thred = surf_dir_first * temp_opposite_dir;
    if (norm_dir * surf_dir_first > in_res_thred &&
        norm_dir * temp_opposite_dir > in_res_thred) {
        return true;
    }
    
    // Check second half range
    temp_opposite_dir = surf_dir_first * (-1.0f);
    in_res_thred = surf_dir_second * temp_opposite_dir;
    if (norm_dir * surf_dir_second > in_res_thred &&
        norm_dir * temp_opposite_dir > in_res_thred) {
        return true;
    }
    
    return false;
}

__device__ bool IsInDirectConstraint_GPU(const NavNode_GPU& node1, const NavNode_GPU& node2) {
    // Check node1 -> node2
    if (node1.free_direct != 3) {  // Not PILLAR
        Point3D_GPU diff_1to2 = node2.position - node1.position;
        if (!IsOutReducedDirs_GPU(diff_1to2, node1.surf_dir_first, node1.surf_dir_second)) {
            return false;
        }
    }
    
    // Check node2 -> node1
    if (node2.free_direct != 3) {  // Not PILLAR
        Point3D_GPU diff_2to1 = node1.position - node2.position;
        if (!IsOutReducedDirs_GPU(diff_2to1, node2.surf_dir_first, node2.surf_dir_second)) {
            return false;
        }
    }
    
    return true;
}

__device__ bool IsConvexConnect_GPU(const NavNode_GPU& node1, const NavNode_GPU& node2) {
    return (node1.free_direct != 2 && node2.free_direct != 2);  // Neither CONCAVE
}

__device__ Point3D_GPU SurfTopoDirect_GPU(const Point3D_GPU& surf_dir_first, 
                                          const Point3D_GPU& surf_dir_second) {
    Point3D_GPU topo_dir = surf_dir_first + surf_dir_second;
    return topo_dir.normalize_flat();
}

__device__ bool IsEdgeCollidePolygons_GPU(const Point3D_GPU& edge_start, const Point3D_GPU& edge_end,
                                          const PolygonEdge_GPU* polygons, int num_polygons,
                                          float height_min, float height_max) {
    for (int i = 0; i < num_polygons; i++) {
        const PolygonEdge_GPU& poly_edge = polygons[i];
        
        // Check height overlap
        if (poly_edge.height_max < height_min || poly_edge.height_min > height_max) {
            continue;
        }
        
        // Check 2D intersection
        if (doIntersect_GPU(edge_start, edge_end, poly_edge.start, poly_edge.end)) {
            return true;
        }
    }
    return false;
}

__device__ bool IsValidConnect_GPU(const NavNode_GPU& node1, const NavNode_GPU& node2,
                                   const PolygonEdge_GPU* polygons, int num_polygons,
                                   float project_dist, float nav_clear_dist) {
    // Check if same node
    if (node1.id == node2.id) return false;
    
    // Check if already connected on contour
    if (IsTypeInStack_GPU(node2.id, node1.contour_connects, node1.num_contour_connects)) {
        return true;
    }
    
    // Check distance
    float dist = (node1.position - node2.position).norm();
    if (dist < nav_clear_dist) {
        return true;  // Very close, assume connected
    }
    
    // Check convexity and direction constraints
    if (!IsConvexConnect_GPU(node1, node2) || !IsInDirectConstraint_GPU(node1, node2)) {
        return false;
    }
    
    // Reproject edge for collision checking
    Point3D_GPU edge_start, edge_end;
    
    // Calculate surface topo direction for node1
    if (node1.free_direct == 1) {  // CONVEX
        Point3D_GPU topo_dir1 = SurfTopoDirect_GPU(node1.surf_dir_first, node1.surf_dir_second);
        edge_start = node1.position - topo_dir1 * project_dist;
    } else if (node1.free_direct == 2) {  // CONCAVE
        Point3D_GPU topo_dir1 = SurfTopoDirect_GPU(node1.surf_dir_first, node1.surf_dir_second);
        edge_start = node1.position + topo_dir1 * project_dist;
    } else {
        edge_start = node1.position;
    }
    
    // Calculate surface topo direction for node2
    if (node2.free_direct == 1) {  // CONVEX
        Point3D_GPU topo_dir2 = SurfTopoDirect_GPU(node2.surf_dir_first, node2.surf_dir_second);
        edge_end = node2.position - topo_dir2 * project_dist;
    } else if (node2.free_direct == 2) {  // CONCAVE
        Point3D_GPU topo_dir2 = SurfTopoDirect_GPU(node2.surf_dir_first, node2.surf_dir_second);
        edge_end = node2.position + topo_dir2 * project_dist;
    } else {
        edge_end = node2.position;
    }
    
    // Check collision with polygon edges
    float height_min = fminf(node1.position.z, node2.position.z);
    float height_max = fmaxf(node1.position.z, node2.position.z);
    
    if (IsEdgeCollidePolygons_GPU(edge_start, edge_end, polygons, num_polygons, height_min, height_max)) {
        return false;
    }
    
    return true;
}

// Main CUDA kernel for visibility checking
__global__ void ComputeVisibilityConnections(const NavNode_GPU* nodes, 
                                              int* connection_matrix,
                                              const PolygonEdge_GPU* polygons,
                                              int num_nodes,
                                              int num_polygons,
                                              float project_dist,
                                              float nav_clear_dist) {
    int i = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;
    
    // Only compute lower triangle (i > j) to avoid duplicates
    if (i >= num_nodes || j >= num_nodes || i <= j) return;
    
    const NavNode_GPU& node1 = nodes[i];
    const NavNode_GPU& node2 = nodes[j];
    
    if (IsValidConnect_GPU(node1, node2, polygons, num_polygons, project_dist, nav_clear_dist)) {
        // Mark connection in matrix (symmetric)
        connection_matrix[i * num_nodes + j] = 1;
        connection_matrix[j * num_nodes + i] = 1;
    }
}

// Host wrapper function
extern "C" {
    void LaunchVisibilityCheckKernel(const NavNode_GPU* d_nodes,
                                     int* d_connection_matrix,
                                     const PolygonEdge_GPU* d_polygons,
                                     int num_nodes,
                                     int num_polygons,
                                     float project_dist,
                                     float nav_clear_dist) {
        // Configure kernel launch parameters
        dim3 block(16, 16);
        dim3 grid((num_nodes + block.x - 1) / block.x, 
                  (num_nodes + block.y - 1) / block.y);
        
        // Launch kernel
        ComputeVisibilityConnections<<<grid, block>>>(
            d_nodes, d_connection_matrix, d_polygons,
            num_nodes, num_polygons, project_dist, nav_clear_dist
        );
        
        // Synchronize and check for errors
        cudaError_t err = cudaDeviceSynchronize();
        if (err != cudaSuccess) {
            printf("CUDA Error: %s\n", cudaGetErrorString(err));
        }
    }
    
    // Helper function to check CUDA availability
    bool IsCudaAvailable() {
        int deviceCount = 0;
        cudaError_t error = cudaGetDeviceCount(&deviceCount);
        if (error != cudaSuccess) {
            printf("[CUDA] cudaGetDeviceCount failed: %s (code %d)\n", 
                   cudaGetErrorString(error), error);
            // Try to get more info
            error = cudaGetLastError(); // Clear error
            return false;
        }
        if (deviceCount == 0) {
            printf("[CUDA] No CUDA devices found (deviceCount = 0)\n");
            return false;
        }
        printf("[CUDA] Successfully detected %d CUDA device(s)\n", deviceCount);
        return true;
    }
    
    // Get GPU device properties
    void GetGPUInfo(int* device_count, char* device_name, int name_len) {
        cudaGetDeviceCount(device_count);
        if (*device_count > 0) {
            cudaDeviceProp prop;
            cudaGetDeviceProperties(&prop, 0);
            snprintf(device_name, name_len, "%s", prop.name);
        }
    }
}
