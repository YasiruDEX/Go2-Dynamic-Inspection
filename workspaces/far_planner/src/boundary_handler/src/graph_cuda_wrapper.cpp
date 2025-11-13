/*
 * CUDA Wrapper Implementation for Visibility Graph GPU Acceleration
 * Copyright (C) 2025 - GPU Acceleration Implementation
 */

#include "boundary_handler/graph_cuda_wrapper.h"
#include <cuda_runtime.h>
#include <iostream>
#include <stdexcept>

namespace GraphCUDA {

CUDAGraphBuilder::CUDAGraphBuilder() : cuda_initialized_(false) {
    InitCUDA();
}

CUDAGraphBuilder::~CUDAGraphBuilder() {
    CleanupCUDA();
}

void CUDAGraphBuilder::InitCUDA() {
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
    
    std::cout << "CUDA initialized successfully. Found " << device_count << " device(s)." << std::endl;
}

void CUDAGraphBuilder::CleanupCUDA() {
    if (cuda_initialized_) {
        cudaDeviceReset();
        cuda_initialized_ = false;
    }
}

bool CUDAGraphBuilder::IsCUDAAvailable() {
    return IsCudaAvailable();
}

std::string CUDAGraphBuilder::GetGPUInfo() {
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

std::vector<std::vector<int>> CUDAGraphBuilder::BuildVisibilityGraph(
    const std::vector<NavNode_GPU>& nodes,
    const std::vector<PolygonVertex_GPU>& polygons,
    float height_tolz) {
    
    if (!cuda_initialized_) {
        throw std::runtime_error("CUDA not initialized");
    }
    
    const int num_nodes = nodes.size();
    const int total_vertices = polygons.size();
    
    if (num_nodes == 0) {
        return std::vector<std::vector<int>>();
    }
    
    // Allocate device memory
    NavNode_GPU* d_nodes = nullptr;
    PolygonVertex_GPU* d_polygons = nullptr;
    int* d_connection_matrix = nullptr;
    
    size_t nodes_size = num_nodes * sizeof(NavNode_GPU);
    size_t polygons_size = total_vertices * sizeof(PolygonVertex_GPU);
    size_t matrix_size = num_nodes * num_nodes * sizeof(int);
    
    cudaError_t err;
    
    err = cudaMalloc(&d_nodes, nodes_size);
    if (err != cudaSuccess) {
        throw std::runtime_error("Failed to allocate device memory for nodes");
    }
    
    err = cudaMalloc(&d_polygons, polygons_size);
    if (err != cudaSuccess) {
        cudaFree(d_nodes);
        throw std::runtime_error("Failed to allocate device memory for polygons");
    }
    
    err = cudaMalloc(&d_connection_matrix, matrix_size);
    if (err != cudaSuccess) {
        cudaFree(d_nodes);
        cudaFree(d_polygons);
        throw std::runtime_error("Failed to allocate device memory for connection matrix");
    }
    
    // Copy data to device
    cudaMemcpy(d_nodes, nodes.data(), nodes_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_polygons, polygons.data(), polygons_size, cudaMemcpyHostToDevice);
    cudaMemset(d_connection_matrix, 0, matrix_size);
    
    // Launch kernel
    LaunchVisibilityGraphKernel(d_nodes, d_connection_matrix, d_polygons,
                                num_nodes, total_vertices, height_tolz);
    
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
    cudaFree(d_polygons);
    cudaFree(d_connection_matrix);
    
    return connection_matrix;
}

} // namespace GraphCUDA
