/***********************************************************
 * CUDA-Accelerated Covariance Matrix Computation for DLIO
 * Optimized for RTX 2060
 ***********************************************************/

#pragma once

#include "nano_gicp/cuda/cuda_knn.cuh"
#include <cuda_runtime.h>
#include <Eigen/Core>
#include <vector>

namespace nano_gicp {
namespace cuda {

// GPU covariance matrix (4x4, row-major)
struct __align__(16) GpuMatrix4 {
    float data[16];
    
    __device__ __host__ GpuMatrix4() {
        for (int i = 0; i < 16; i++) data[i] = 0.0f;
    }
    
    __device__ __host__ void setIdentity() {
        for (int i = 0; i < 16; i++) data[i] = 0.0f;
        data[0] = data[5] = data[10] = data[15] = 1.0f;
    }
};

class CudaCovarianceCalculator {
public:
    CudaCovarianceCalculator();
    ~CudaCovarianceCalculator();
    
    // Calculate covariances for all points in the cloud
    // points: input point cloud [x,y,z, x,y,z, ...]
    // num_points: number of points
    // knn_search: KNN search object (already has target cloud set)
    // k: number of neighbors for covariance computation
    // covariances: output array of 4x4 covariance matrices (in Eigen format)
    // density: output average density metric
    bool calculateCovariances(const float* points, int num_points,
                             CudaKNNSearch& knn_search, int k,
                             std::vector<Eigen::Matrix4d>& covariances,
                             float& density);
    
    void clear();
    
private:
    // GPU memory
    GpuPoint* d_points_;
    int* d_knn_indices_;
    float* d_knn_distances_;
    GpuMatrix4* d_covariances_;
    float* d_density_contributions_;
    
    int max_points_;
    int max_k_;
    bool initialized_;
    
    void allocateMemory(int max_points, int k);
};

// CUDA kernel declarations
__global__ void computeCovariancesKernel(
    const GpuPoint* points, int num_points,
    const int* knn_indices, const float* knn_distances,
    int k, GpuMatrix4* covariances,
    float* density_contributions);

__global__ void regularizeCovariancesKernel(
    GpuMatrix4* covariances, int num_points);

// Device helper functions
__device__ void computeSVD3x3(const float* A, float* U, float* S, float* V);
__device__ void matrixMultiply3x3(const float* A, const float* B, float* C);

} // namespace cuda
} // namespace nano_gicp
