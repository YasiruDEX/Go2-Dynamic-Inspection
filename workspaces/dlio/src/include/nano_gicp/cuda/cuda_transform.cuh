/***********************************************************
 * CUDA-Accelerated Point Cloud Transformations for DLIO
 * Optimized for RTX 2060
 ***********************************************************/

#pragma once

#include "nano_gicp/cuda/cuda_knn.cuh"
#include <cuda_runtime.h>
#include <Eigen/Core>

namespace nano_gicp {
namespace cuda {

class CudaPointCloudTransform {
public:
    CudaPointCloudTransform();
    ~CudaPointCloudTransform();
    
    // Transform point cloud with 4x4 transformation matrix
    // points: input/output point cloud [x,y,z, x,y,z, ...]
    // num_points: number of points
    // transform: 4x4 transformation matrix (row-major)
    void transformPointCloud(float* points, int num_points, const float* transform);
    
    // Batch transform multiple point clouds (useful for deskewing)
    void batchTransformPointCloud(float* points, int num_points,
                                 const float* transforms, int num_transforms,
                                 const int* point_to_transform_map);
    
    void clear();
    
private:
    GpuPoint* d_points_;
    float* d_transform_;
    int max_points_;
    bool initialized_;
    
    void allocateMemory(int max_points);
};

// CUDA kernel declarations
__global__ void transformPointsKernel(GpuPoint* points, int num_points,
                                     const float* transform);

__global__ void batchTransformPointsKernel(GpuPoint* points, int num_points,
                                          const float* transforms, int num_transforms,
                                          const int* point_to_transform_map);

// Device helper for matrix-vector multiplication
__device__ inline GpuPoint transformPoint(const GpuPoint& p, const float* T) {
    GpuPoint result;
    result.x = T[0] * p.x + T[1] * p.y + T[2]  * p.z + T[3];
    result.y = T[4] * p.x + T[5] * p.y + T[6]  * p.z + T[7];
    result.z = T[8] * p.x + T[9] * p.y + T[10] * p.z + T[11];
    result.w = 1.0f;
    return result;
}

} // namespace cuda
} // namespace nano_gicp
