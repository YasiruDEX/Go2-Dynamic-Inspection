/*
 * CUDA Wrapper Implementation for FAR Planner GPU Acceleration
 * Copyright (C) 2025 - GPU Acceleration Implementation
 */

#include "far_planner/far_planner_cuda_wrapper.h"
#include <cuda_runtime.h>
#include <iostream>
#include <stdexcept>

namespace FARPlannerCUDA {

CUDAVisibilityChecker::CUDAVisibilityChecker() : cuda_initialized_(false) {
    InitCUDA();
}

CUDAVisibilityChecker::~CUDAVisibilityChecker() {
    CleanupCUDA();
}

void CUDAVisibilityChecker::InitCUDA() {
    if (cuda_initialized_) return;
    
    int device_count = 0;
    cudaError_t error = cudaGetDeviceCount(&device_count);
    
    if (error != cudaSuccess || device_count == 0) {
        std::cerr << "CUDA initialization failed: " << cudaGetErrorString(error) << std::endl;
        cuda_initialized_ = false;
        return;
    }
    
    // Set device 0 as default
    cudaSetDevice(0);
    cuda_initialized_ = true;
    
    std::cout << "FAR Planner CUDA initialized successfully. Found " << device_count << " device(s)." << std::endl;
}

void CUDAVisibilityChecker::CleanupCUDA() {
    if (cuda_initialized_) {
        cudaDeviceReset();
        cuda_initialized_ = false;
    }
}

bool CUDAVisibilityChecker::IsCUDAAvailable() {
    return IsCudaAvailable();
}

std::string CUDAVisibilityChecker::GetGPUInfo() {
    int device_count = 0;
    char device_name[256] = "Unknown";
    
    cudaGetDeviceCount(&device_count);
    if (device_count > 0) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
        snprintf(device_name, sizeof(device_name), "%s", prop.name);
    }
    
    return "GPU: " + std::string(device_name) + " (Count: " + std::to_string(device_count) + ")";
}

NavNode_GPU CUDAVisibilityChecker::ConvertToGPUNode(const NavNodePtr& node) {
    NavNode_GPU gpu_node;
    gpu_node.position = Point3D_GPU(node->position.x, node->position.y, node->position.z);
    gpu_node.id = node->id;
    gpu_node.free_direct = static_cast<int>(node->free_direct);
    gpu_node.surf_dir_first = Point3D_GPU(node->surf_dirs.first.x, 
                                           node->surf_dirs.first.y, 
                                           node->surf_dirs.first.z);
    gpu_node.surf_dir_second = Point3D_GPU(node->surf_dirs.second.x, 
                                            node->surf_dirs.second.y, 
                                            node->surf_dirs.second.z);
    gpu_node.is_boundary = node->is_boundary;
    gpu_node.is_odom = node->is_odom;
    gpu_node.is_navpoint = node->is_navpoint;
    
    // Copy contour connections - extract IDs from NavNodePtr stack
    gpu_node.num_contour_connects = std::min((int)node->contour_connects.size(), 50);
    for (int j = 0; j < gpu_node.num_contour_connects; j++) {
        gpu_node.contour_connects[j] = node->contour_connects[j]->id;
    }
    
    return gpu_node;
}

PolygonEdge_GPU CUDAVisibilityChecker::ConvertToGPUEdge(const PointPair& edge) {
    PolygonEdge_GPU gpu_edge;
    gpu_edge.start = Point3D_GPU(edge.first.x, edge.first.y, edge.first.z);
    gpu_edge.end = Point3D_GPU(edge.second.x, edge.second.y, edge.second.z);
    gpu_edge.polygon_id = 0;  // Will be set by caller if needed
    gpu_edge.height_min = std::min(edge.first.z, edge.second.z);
    gpu_edge.height_max = std::max(edge.first.z, edge.second.z);
    gpu_edge.is_boundary = true;  // Assuming contour edges are boundaries
    
    return gpu_edge;
}

std::vector<std::vector<int>> CUDAVisibilityChecker::CheckVisibilityConnections(
    const std::vector<NavNodePtr>& nodes,
    const std::vector<PointPair>& contour_edges,
    float project_dist,
    float nav_clear_dist)
{
    if (!cuda_initialized_) {
        throw std::runtime_error("CUDA not initialized");
    }
    
    const int num_nodes = nodes.size();
    const int num_edges = contour_edges.size();
    
    if (num_nodes == 0) {
        return std::vector<std::vector<int>>();
    }
    
    // Convert nodes to GPU format
    std::vector<NavNode_GPU> gpu_nodes(num_nodes);
    for (int i = 0; i < num_nodes; i++) {
        gpu_nodes[i] = ConvertToGPUNode(nodes[i]);
    }
    
    // Convert edges to GPU format
    std::vector<PolygonEdge_GPU> gpu_edges(num_edges);
    for (int i = 0; i < num_edges; i++) {
        gpu_edges[i] = ConvertToGPUEdge(contour_edges[i]);
    }
    
    // Allocate device memory
    NavNode_GPU* d_nodes = nullptr;
    PolygonEdge_GPU* d_edges = nullptr;
    int* d_connection_matrix = nullptr;
    
    size_t nodes_size = num_nodes * sizeof(NavNode_GPU);
    size_t edges_size = num_edges * sizeof(PolygonEdge_GPU);
    size_t matrix_size = num_nodes * num_nodes * sizeof(int);
    
    cudaError_t err;
    
    err = cudaMalloc(&d_nodes, nodes_size);
    if (err != cudaSuccess) {
        char error_msg[256];
        snprintf(error_msg, sizeof(error_msg), 
                "Failed to allocate device memory for nodes (size: %.2f MB): %s", 
                nodes_size / (1024.0 * 1024.0), cudaGetErrorString(err));
        throw std::runtime_error(error_msg);
    }
    
    err = cudaMalloc(&d_edges, edges_size);
    if (err != cudaSuccess) {
        cudaFree(d_nodes);
        char error_msg[256];
        snprintf(error_msg, sizeof(error_msg), 
                "Failed to allocate device memory for edges (size: %.2f MB): %s", 
                edges_size / (1024.0 * 1024.0), cudaGetErrorString(err));
        throw std::runtime_error(error_msg);
    }
    
    err = cudaMalloc(&d_connection_matrix, matrix_size);
    if (err != cudaSuccess) {
        cudaFree(d_nodes);
        cudaFree(d_edges);
        char error_msg[256];
        snprintf(error_msg, sizeof(error_msg), 
                "Failed to allocate device memory for connection matrix (size: %.2f MB): %s", 
                matrix_size / (1024.0 * 1024.0), cudaGetErrorString(err));
        throw std::runtime_error(error_msg);
    }
    
    // Copy data to device
    cudaMemcpy(d_nodes, gpu_nodes.data(), nodes_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_edges, gpu_edges.data(), edges_size, cudaMemcpyHostToDevice);
    cudaMemset(d_connection_matrix, 0, matrix_size);
    
    // Launch kernel
    LaunchVisibilityCheckKernel(d_nodes, d_connection_matrix, d_edges,
                                num_nodes, num_edges, project_dist, nav_clear_dist);
    
    // Copy results back
    std::vector<int> connection_matrix_flat(num_nodes * num_nodes);
    cudaMemcpy(connection_matrix_flat.data(), d_connection_matrix, 
               matrix_size, cudaMemcpyDeviceToHost);
    
    // Convert to 2D vector
    std::vector<std::vector<int>> connection_matrix(num_nodes, std::vector<int>(num_nodes, 0));
    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            connection_matrix[i][j] = connection_matrix_flat[i * num_nodes + j];
        }
    }
    
    // Free device memory
    cudaFree(d_nodes);
    cudaFree(d_edges);
    cudaFree(d_connection_matrix);
    
    return connection_matrix;
}

} // namespace FARPlannerCUDA
