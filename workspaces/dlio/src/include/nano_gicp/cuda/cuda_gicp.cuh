/***********************************************************
 * CUDA-Accelerated GICP Operations for DLIO
 * Optimized for RTX 2060
 ***********************************************************/

#pragma once

#include "nano_gicp/cuda/cuda_knn.cuh"
#include "nano_gicp/cuda/cuda_covariance.cuh"
#include <cuda_runtime.h>
#include <Eigen/Core>

namespace nano_gicp {
namespace cuda {

class CudaGICP {
public:
    CudaGICP();
    ~CudaGICP();
    
    // Update correspondences between source and target clouds
    void updateCorrespondences(
        const float* source_points, int num_source,
        const float* target_points, int num_target,
        const GpuMatrix4* source_covs,
        const GpuMatrix4* target_covs,
        const float* transform, // 4x4 matrix
        float max_correspondence_distance,
        int* correspondences,  // Output: size num_source
        float* sq_distances,   // Output: size num_source
        GpuMatrix4* mahalanobis); // Output: size num_source
    
    // Compute linearization (H and b for LM optimization)
    double computeLinearization(
        const float* source_points, int num_source,
        const float* target_points, int num_target,
        const int* correspondences,
        const GpuMatrix4* mahalanobis,
        const float* transform,
        Eigen::Matrix<double, 6, 6>& H,
        Eigen::Matrix<double, 6, 1>& b);
    
    // Compute error only (no Jacobian)
    double computeError(
        const float* source_points, int num_source,
        const float* target_points, int num_target,
        const int* correspondences,
        const GpuMatrix4* mahalanobis,
        const float* transform);
    
    void clear();
    
private:
    // GPU memory
    GpuPoint* d_source_points_;
    GpuPoint* d_target_points_;
    GpuMatrix4* d_source_covs_;
    GpuMatrix4* d_target_covs_;
    int* d_correspondences_;
    float* d_sq_distances_;
    GpuMatrix4* d_mahalanobis_;
    float* d_transform_;
    
    // For linearization
    double* d_H_blocks_;  // Per-thread H matrices
    double* d_b_blocks_;  // Per-thread b vectors
    double* d_errors_;    // Per-thread errors
    
    CudaKNNSearch knn_search_;
    
    int max_source_points_;
    int max_target_points_;
    bool initialized_;
    
    void allocateMemory(int max_source, int max_target);
};

// CUDA kernel declarations
__global__ void updateCorrespondencesKernel(
    const GpuPoint* source_points, int num_source,
    const GpuPoint* target_points, int num_target,
    const GpuMatrix4* source_covs,
    const GpuMatrix4* target_covs,
    const float* transform,
    float max_corr_dist_sq,
    const int* knn_indices,
    const float* knn_sq_dists,
    int* correspondences,
    float* sq_distances,
    GpuMatrix4* mahalanobis);

__global__ void computeLinearizationKernel(
    const GpuPoint* source_points, int num_source,
    const GpuPoint* target_points,
    const int* correspondences,
    const GpuMatrix4* mahalanobis,
    const float* transform,
    double* H_blocks,  // Output: 6x6 matrices per thread
    double* b_blocks,  // Output: 6x1 vectors per thread
    double* errors);   // Output: scalar per thread

__global__ void computeErrorKernel(
    const GpuPoint* source_points, int num_source,
    const GpuPoint* target_points,
    const int* correspondences,
    const GpuMatrix4* mahalanobis,
    const float* transform,
    double* errors);

// Device helper functions
__device__ void skewSymmetric(const double* v, double* S);
__device__ void matrix4VectorMultiply(const GpuMatrix4& M, const double* v, double* result);

} // namespace cuda
} // namespace nano_gicp
