/***********************************************************
 * CUDA-Accelerated Point Cloud Transformations Implementation
 ***********************************************************/

#include "nano_gicp/cuda/cuda_transform.cuh"
#include <algorithm>

namespace nano_gicp {
namespace cuda {

__global__ void transformPointsKernel(GpuPoint* points, int num_points,
                                     const float* transform) {
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    
    points[idx] = transformPoint(points[idx], transform);
}

__global__ void batchTransformPointsKernel(GpuPoint* points, int num_points,
                                          const float* transforms, int num_transforms,
                                          const int* point_to_transform_map) {
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    
    int transform_idx = point_to_transform_map[idx];
    if (transform_idx >= 0 && transform_idx < num_transforms) {
        const float* T = transforms + transform_idx * 16;
        points[idx] = transformPoint(points[idx], T);
    }
}

CudaPointCloudTransform::CudaPointCloudTransform()
    : d_points_(nullptr)
    , d_transform_(nullptr)
    , max_points_(0)
    , initialized_(false) {
}

CudaPointCloudTransform::~CudaPointCloudTransform() {
    clear();
}

void CudaPointCloudTransform::allocateMemory(int max_points) {
    if (initialized_ && max_points <= max_points_) {
        return;
    }
    
    clear();
    
    max_points_ = max_points;
    
    CUDA_CHECK(cudaMalloc(&d_points_, max_points * sizeof(GpuPoint)));
    CUDA_CHECK(cudaMalloc(&d_transform_, 16 * sizeof(float)));
    
    initialized_ = true;
}

void CudaPointCloudTransform::transformPointCloud(float* points, int num_points,
                                                  const float* transform) {
    allocateMemory(num_points);
    
    // Convert and copy points to GPU
    std::vector<GpuPoint> h_points(num_points);
    for (int i = 0; i < num_points; i++) {
        h_points[i].x = points[i * 3 + 0];
        h_points[i].y = points[i * 3 + 1];
        h_points[i].z = points[i * 3 + 2];
        h_points[i].w = 1.0f;
    }
    
    CUDA_CHECK(cudaMemcpy(d_points_, h_points.data(),
                         num_points * sizeof(GpuPoint), cudaMemcpyHostToDevice));
    
    // Copy transform matrix (assume row-major 4x4)
    CUDA_CHECK(cudaMemcpy(d_transform_, transform, 16 * sizeof(float),
                         cudaMemcpyHostToDevice));
    
    // Launch kernel
    const int THREADS_PER_BLOCK = 256;
    const int NUM_BLOCKS = (num_points + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    transformPointsKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
        d_points_, num_points, d_transform_
    );
    
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Copy results back
    CUDA_CHECK(cudaMemcpy(h_points.data(), d_points_,
                         num_points * sizeof(GpuPoint), cudaMemcpyDeviceToHost));
    
    for (int i = 0; i < num_points; i++) {
        points[i * 3 + 0] = h_points[i].x;
        points[i * 3 + 1] = h_points[i].y;
        points[i * 3 + 2] = h_points[i].z;
    }
}

void CudaPointCloudTransform::batchTransformPointCloud(
    float* points, int num_points,
    const float* transforms, int num_transforms,
    const int* point_to_transform_map) {
    
    allocateMemory(num_points);
    
    // Convert and copy points
    std::vector<GpuPoint> h_points(num_points);
    for (int i = 0; i < num_points; i++) {
        h_points[i].x = points[i * 3 + 0];
        h_points[i].y = points[i * 3 + 1];
        h_points[i].z = points[i * 3 + 2];
        h_points[i].w = 1.0f;
    }
    
    CUDA_CHECK(cudaMemcpy(d_points_, h_points.data(),
                         num_points * sizeof(GpuPoint), cudaMemcpyHostToDevice));
    
    // Allocate and copy transforms
    float* d_transforms;
    CUDA_CHECK(cudaMalloc(&d_transforms, num_transforms * 16 * sizeof(float)));
    CUDA_CHECK(cudaMemcpy(d_transforms, transforms,
                         num_transforms * 16 * sizeof(float), cudaMemcpyHostToDevice));
    
    // Allocate and copy mapping
    int* d_map;
    CUDA_CHECK(cudaMalloc(&d_map, num_points * sizeof(int)));
    CUDA_CHECK(cudaMemcpy(d_map, point_to_transform_map,
                         num_points * sizeof(int), cudaMemcpyHostToDevice));
    
    // Launch kernel
    const int THREADS_PER_BLOCK = 256;
    const int NUM_BLOCKS = (num_points + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    batchTransformPointsKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
        d_points_, num_points, d_transforms, num_transforms, d_map
    );
    
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Copy results back
    CUDA_CHECK(cudaMemcpy(h_points.data(), d_points_,
                         num_points * sizeof(GpuPoint), cudaMemcpyDeviceToHost));
    
    for (int i = 0; i < num_points; i++) {
        points[i * 3 + 0] = h_points[i].x;
        points[i * 3 + 1] = h_points[i].y;
        points[i * 3 + 2] = h_points[i].z;
    }
    
    // Cleanup
    CUDA_CHECK(cudaFree(d_transforms));
    CUDA_CHECK(cudaFree(d_map));
}

void CudaPointCloudTransform::clear() {
    if (d_points_) {
        CUDA_CHECK(cudaFree(d_points_));
        d_points_ = nullptr;
    }
    if (d_transform_) {
        CUDA_CHECK(cudaFree(d_transform_));
        d_transform_ = nullptr;
    }
    
    max_points_ = 0;
    initialized_ = false;
}

} // namespace cuda
} // namespace nano_gicp
