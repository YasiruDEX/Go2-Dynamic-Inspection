/***********************************************************
 * CUDA-Accelerated GICP Implementation
 ***********************************************************/

#include "nano_gicp/cuda/cuda_gicp.cuh"
#include <algorithm>
#include <cfloat>

namespace nano_gicp {
namespace cuda {

__device__ void skewSymmetric(const double* v, double* S) {
    S[0] = 0.0;    S[1] = -v[2];  S[2] = v[1];
    S[3] = v[2];   S[4] = 0.0;    S[5] = -v[0];
    S[6] = -v[1];  S[7] = v[0];   S[8] = 0.0;
}

__device__ void matrix4VectorMultiply(const GpuMatrix4& M, const double* v, double* result) {
    result[0] = M.data[0] * v[0] + M.data[1] * v[1] + M.data[2]  * v[2] + M.data[3]  * v[3];
    result[1] = M.data[4] * v[0] + M.data[5] * v[1] + M.data[6]  * v[2] + M.data[7]  * v[3];
    result[2] = M.data[8] * v[0] + M.data[9] * v[1] + M.data[10] * v[2] + M.data[11] * v[3];
    result[3] = M.data[12] * v[0] + M.data[13] * v[1] + M.data[14] * v[2] + M.data[15] * v[3];
}

__device__ void invertMatrix4(const GpuMatrix4& M, GpuMatrix4& M_inv) {
    // Simplified inversion for 4x4 matrix (assuming bottom row is [0,0,0,1])
    // Only invert the 3x3 rotation part and adjust translation
    
    // Extract 3x3 part
    double R[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i * 3 + j] = M.data[i * 4 + j];
        }
    }
    
    // Compute determinant
    double det = R[0] * (R[4] * R[8] - R[5] * R[7])
               - R[1] * (R[3] * R[8] - R[5] * R[6])
               + R[2] * (R[3] * R[7] - R[4] * R[6]);
    
    if (fabs(det) < 1e-10) {
        // Singular matrix, return identity
        M_inv.setIdentity();
        return;
    }
    
    double inv_det = 1.0 / det;
    
    // Compute inverse of 3x3 part
    double R_inv[9];
    R_inv[0] = (R[4] * R[8] - R[5] * R[7]) * inv_det;
    R_inv[1] = (R[2] * R[7] - R[1] * R[8]) * inv_det;
    R_inv[2] = (R[1] * R[5] - R[2] * R[4]) * inv_det;
    R_inv[3] = (R[5] * R[6] - R[3] * R[8]) * inv_det;
    R_inv[4] = (R[0] * R[8] - R[2] * R[6]) * inv_det;
    R_inv[5] = (R[2] * R[3] - R[0] * R[5]) * inv_det;
    R_inv[6] = (R[3] * R[7] - R[4] * R[6]) * inv_det;
    R_inv[7] = (R[1] * R[6] - R[0] * R[7]) * inv_det;
    R_inv[8] = (R[0] * R[4] - R[1] * R[3]) * inv_det;
    
    // Build output matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            M_inv.data[i * 4 + j] = R_inv[i * 3 + j];
        }
    }
    
    // Compute -R^(-1) * t
    double t[3] = {M.data[3], M.data[7], M.data[11]};
    M_inv.data[3]  = -(R_inv[0] * t[0] + R_inv[1] * t[1] + R_inv[2] * t[2]);
    M_inv.data[7]  = -(R_inv[3] * t[0] + R_inv[4] * t[1] + R_inv[5] * t[2]);
    M_inv.data[11] = -(R_inv[6] * t[0] + R_inv[7] * t[1] + R_inv[8] * t[2]);
    
    // Bottom row
    M_inv.data[12] = M_inv.data[13] = M_inv.data[14] = 0.0f;
    M_inv.data[15] = 1.0f;
}

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
    GpuMatrix4* mahalanobis) {
    
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_source) return;
    
    // Transform source point
    GpuPoint src = source_points[idx];
    GpuPoint transformed = transformPoint(src, transform);
    
    // Get nearest neighbor from KNN search results
    int nearest_idx = knn_indices[idx];
    float sq_dist = knn_sq_dists[idx];
    
    // Check if correspondence is valid
    if (nearest_idx < 0 || sq_dist > max_corr_dist_sq) {
        correspondences[idx] = -1;
        sq_distances[idx] = FLT_MAX;
        return;
    }
    
    correspondences[idx] = nearest_idx;
    sq_distances[idx] = sq_dist;
    
    // Compute Mahalanobis distance matrix
    // M = (cov_B + T * cov_A * T^T)^(-1)
    
    const GpuMatrix4& cov_A = source_covs[idx];
    const GpuMatrix4& cov_B = target_covs[nearest_idx];
    
    // T * cov_A
    GpuMatrix4 T_cov_A;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += transform[i * 4 + k] * cov_A.data[k * 4 + j];
            }
            T_cov_A.data[i * 4 + j] = sum;
        }
    }
    
    // T * cov_A * T^T
    GpuMatrix4 RCR;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += T_cov_A.data[i * 4 + k] * transform[j * 4 + k];
            }
            RCR.data[i * 4 + j] = sum;
        }
    }
    
    // RCR = cov_B + RCR
    for (int i = 0; i < 16; i++) {
        RCR.data[i] += cov_B.data[i];
    }
    RCR.data[15] = 1.0f;
    
    // Invert RCR
    GpuMatrix4 RCR_inv;
    invertMatrix4(RCR, RCR_inv);
    
    // Set last element to 0 as per original code
    RCR_inv.data[15] = 0.0f;
    
    mahalanobis[idx] = RCR_inv;
}

__global__ void computeLinearizationKernel(
    const GpuPoint* source_points, int num_source,
    const GpuPoint* target_points,
    const int* correspondences,
    const GpuMatrix4* mahalanobis,
    const float* transform,
    double* H_blocks,
    double* b_blocks,
    double* errors) {
    
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_source) return;
    
    int target_idx = correspondences[idx];
    if (target_idx < 0) {
        // No valid correspondence
        errors[idx] = 0.0;
        // H and b blocks are already zero-initialized
        return;
    }
    
    // Get points
    GpuPoint src = source_points[idx];
    GpuPoint tgt = target_points[target_idx];
    
    // Transform source point
    double mean_A[4] = {src.x, src.y, src.z, 1.0};
    double mean_B[4] = {tgt.x, tgt.y, tgt.z, 1.0};
    
    // transed_mean_A = T * mean_A
    double transed_mean_A[4];
    transed_mean_A[0] = transform[0] * mean_A[0] + transform[1] * mean_A[1] + 
                        transform[2] * mean_A[2] + transform[3];
    transed_mean_A[1] = transform[4] * mean_A[0] + transform[5] * mean_A[1] + 
                        transform[6] * mean_A[2] + transform[7];
    transed_mean_A[2] = transform[8] * mean_A[0] + transform[9] * mean_A[1] + 
                        transform[10] * mean_A[2] + transform[11];
    transed_mean_A[3] = 1.0;
    
    // error = mean_B - transed_mean_A
    double error[4];
    for (int i = 0; i < 4; i++) {
        error[i] = mean_B[i] - transed_mean_A[i];
    }
    
    // Compute error: e^T * M * e
    double M_error[4];
    matrix4VectorMultiply(mahalanobis[idx], error, M_error);
    
    double error_val = 0.0;
    for (int i = 0; i < 4; i++) {
        error_val += error[i] * M_error[i];
    }
    errors[idx] = error_val;
    
    // Compute Jacobian: dtdx0 = [skew(transed_mean_A), -I; 0, 0]
    // dtdx0 is 4x6 matrix
    double dtdx0[24] = {0.0}; // 4 rows x 6 cols
    
    // First 3x3 block: skew(transed_mean_A)
    double skew[9];
    skewSymmetric(transed_mean_A, skew);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dtdx0[i * 6 + j] = skew[i * 3 + j];
        }
    }
    
    // Second 3x3 block: -I
    dtdx0[0 * 6 + 3] = -1.0;
    dtdx0[1 * 6 + 4] = -1.0;
    dtdx0[2 * 6 + 5] = -1.0;
    
    // Compute jlossexp = dtdx0
    // Compute Hi = jlossexp^T * M * jlossexp (6x6 matrix)
    
    // First: M * jlossexp (4x6)
    double M_jlossexp[24];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) {
                sum += mahalanobis[idx].data[i * 4 + k] * dtdx0[k * 6 + j];
            }
            M_jlossexp[i * 6 + j] = sum;
        }
    }
    
    // Hi = dtdx0^T * M_jlossexp (6x6)
    int H_offset = idx * 36; // 6x6 = 36 elements
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) {
                sum += dtdx0[k * 6 + i] * M_jlossexp[k * 6 + j];
            }
            H_blocks[H_offset + i * 6 + j] = sum;
        }
    }
    
    // bi = jlossexp^T * M * error (6x1)
    int b_offset = idx * 6;
    for (int i = 0; i < 6; i++) {
        double sum = 0.0;
        for (int k = 0; k < 4; k++) {
            sum += dtdx0[k * 6 + i] * M_error[k];
        }
        b_blocks[b_offset + i] = sum;
    }
}

__global__ void computeErrorKernel(
    const GpuPoint* source_points, int num_source,
    const GpuPoint* target_points,
    const int* correspondences,
    const GpuMatrix4* mahalanobis,
    const float* transform,
    double* errors) {
    
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_source) return;
    
    int target_idx = correspondences[idx];
    if (target_idx < 0) {
        errors[idx] = 0.0;
        return;
    }
    
    GpuPoint src = source_points[idx];
    GpuPoint tgt = target_points[target_idx];
    
    double mean_A[4] = {src.x, src.y, src.z, 1.0};
    double mean_B[4] = {tgt.x, tgt.y, tgt.z, 1.0};
    
    double transed_mean_A[4];
    transed_mean_A[0] = transform[0] * mean_A[0] + transform[1] * mean_A[1] + 
                        transform[2] * mean_A[2] + transform[3];
    transed_mean_A[1] = transform[4] * mean_A[0] + transform[5] * mean_A[1] + 
                        transform[6] * mean_A[2] + transform[7];
    transed_mean_A[2] = transform[8] * mean_A[0] + transform[9] * mean_A[1] + 
                        transform[10] * mean_A[2] + transform[11];
    transed_mean_A[3] = 1.0;
    
    double error[4];
    for (int i = 0; i < 4; i++) {
        error[i] = mean_B[i] - transed_mean_A[i];
    }
    
    double M_error[4];
    matrix4VectorMultiply(mahalanobis[idx], error, M_error);
    
    double error_val = 0.0;
    for (int i = 0; i < 4; i++) {
        error_val += error[i] * M_error[i];
    }
    errors[idx] = error_val;
}

// CudaGICP implementation
CudaGICP::CudaGICP()
    : d_source_points_(nullptr)
    , d_target_points_(nullptr)
    , d_source_covs_(nullptr)
    , d_target_covs_(nullptr)
    , d_correspondences_(nullptr)
    , d_sq_distances_(nullptr)
    , d_mahalanobis_(nullptr)
    , d_transform_(nullptr)
    , d_H_blocks_(nullptr)
    , d_b_blocks_(nullptr)
    , d_errors_(nullptr)
    , max_source_points_(0)
    , max_target_points_(0)
    , initialized_(false) {
}

CudaGICP::~CudaGICP() {
    clear();
}

void CudaGICP::allocateMemory(int max_source, int max_target) {
    if (initialized_ && max_source <= max_source_points_ && max_target <= max_target_points_) {
        return;
    }
    
    clear();
    
    max_source_points_ = max_source;
    max_target_points_ = max_target;
    
    CUDA_CHECK(cudaMalloc(&d_source_points_, max_source * sizeof(GpuPoint)));
    CUDA_CHECK(cudaMalloc(&d_target_points_, max_target * sizeof(GpuPoint)));
    CUDA_CHECK(cudaMalloc(&d_source_covs_, max_source * sizeof(GpuMatrix4)));
    CUDA_CHECK(cudaMalloc(&d_target_covs_, max_target * sizeof(GpuMatrix4)));
    CUDA_CHECK(cudaMalloc(&d_correspondences_, max_source * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_sq_distances_, max_source * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_mahalanobis_, max_source * sizeof(GpuMatrix4)));
    CUDA_CHECK(cudaMalloc(&d_transform_, 16 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_H_blocks_, max_source * 36 * sizeof(double))); // 6x6 per point
    CUDA_CHECK(cudaMalloc(&d_b_blocks_, max_source * 6 * sizeof(double)));  // 6x1 per point
    CUDA_CHECK(cudaMalloc(&d_errors_, max_source * sizeof(double)));
    
    // Initialize to zero
    CUDA_CHECK(cudaMemset(d_H_blocks_, 0, max_source * 36 * sizeof(double)));
    CUDA_CHECK(cudaMemset(d_b_blocks_, 0, max_source * 6 * sizeof(double)));
    
    initialized_ = true;
}

void CudaGICP::updateCorrespondences(
    const float* source_points, int num_source,
    const float* target_points, int num_target,
    const GpuMatrix4* source_covs,
    const GpuMatrix4* target_covs,
    const float* transform,
    float max_correspondence_distance,
    int* correspondences,
    float* sq_distances,
    GpuMatrix4* mahalanobis) {
    
    allocateMemory(num_source, num_target);
    
    // Convert and copy points
    std::vector<GpuPoint> h_source(num_source), h_target(num_target);
    for (int i = 0; i < num_source; i++) {
        h_source[i].x = source_points[i * 3 + 0];
        h_source[i].y = source_points[i * 3 + 1];
        h_source[i].z = source_points[i * 3 + 2];
        h_source[i].w = 1.0f;
    }
    for (int i = 0; i < num_target; i++) {
        h_target[i].x = target_points[i * 3 + 0];
        h_target[i].y = target_points[i * 3 + 1];
        h_target[i].z = target_points[i * 3 + 2];
        h_target[i].w = 1.0f;
    }
    
    CUDA_CHECK(cudaMemcpy(d_source_points_, h_source.data(),
                         num_source * sizeof(GpuPoint), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_target_points_, h_target.data(),
                         num_target * sizeof(GpuPoint), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_source_covs_, source_covs,
                         num_source * sizeof(GpuMatrix4), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_target_covs_, target_covs,
                         num_target * sizeof(GpuMatrix4), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_transform_, transform, 16 * sizeof(float),
                         cudaMemcpyHostToDevice));
    
    // Use KNN search to find nearest neighbors for transformed source points
    std::vector<float> transformed_source(num_source * 3);
    for (int i = 0; i < num_source; i++) {
        GpuPoint pt = transformPoint(h_source[i], transform);
        transformed_source[i * 3 + 0] = pt.x;
        transformed_source[i * 3 + 1] = pt.y;
        transformed_source[i * 3 + 2] = pt.z;
    }
    
    std::vector<float> target_flat(num_target * 3);
    for (int i = 0; i < num_target; i++) {
        target_flat[i * 3 + 0] = h_target[i].x;
        target_flat[i * 3 + 1] = h_target[i].y;
        target_flat[i * 3 + 2] = h_target[i].z;
    }
    
    knn_search_.setTargetCloud(target_flat.data(), num_target);
    
    std::vector<int> knn_indices(num_source);
    std::vector<float> knn_sq_dists(num_source);
    knn_search_.batchKnnSearch(transformed_source.data(), num_source, 1,
                              knn_indices.data(), knn_sq_dists.data());
    
    // Copy KNN results to GPU
    int* d_knn_indices;
    float* d_knn_sq_dists;
    CUDA_CHECK(cudaMalloc(&d_knn_indices, num_source * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_knn_sq_dists, num_source * sizeof(float)));
    CUDA_CHECK(cudaMemcpy(d_knn_indices, knn_indices.data(),
                         num_source * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_knn_sq_dists, knn_sq_dists.data(),
                         num_source * sizeof(float), cudaMemcpyHostToDevice));
    
    // Launch kernel
    const int THREADS_PER_BLOCK = 256;
    const int NUM_BLOCKS = (num_source + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    float max_corr_dist_sq = max_correspondence_distance * max_correspondence_distance;
    
    updateCorrespondencesKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
        d_source_points_, num_source,
        d_target_points_, num_target,
        d_source_covs_, d_target_covs_,
        d_transform_, max_corr_dist_sq,
        d_knn_indices, d_knn_sq_dists,
        d_correspondences_, d_sq_distances_, d_mahalanobis_
    );
    
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Copy results back
    CUDA_CHECK(cudaMemcpy(correspondences, d_correspondences_,
                         num_source * sizeof(int), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(sq_distances, d_sq_distances_,
                         num_source * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(mahalanobis, d_mahalanobis_,
                         num_source * sizeof(GpuMatrix4), cudaMemcpyDeviceToHost));
    
    // Cleanup
    CUDA_CHECK(cudaFree(d_knn_indices));
    CUDA_CHECK(cudaFree(d_knn_sq_dists));
}

double CudaGICP::computeLinearization(
    const float* source_points, int num_source,
    const float* target_points, int num_target,
    const int* correspondences,
    const GpuMatrix4* mahalanobis,
    const float* transform,
    Eigen::Matrix<double, 6, 6>& H,
    Eigen::Matrix<double, 6, 1>& b) {
    
    // Copy data to GPU (points should already be there from updateCorrespondences)
    CUDA_CHECK(cudaMemcpy(d_correspondences_, correspondences,
                         num_source * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_mahalanobis_, mahalanobis,
                         num_source * sizeof(GpuMatrix4), cudaMemcpyHostToDevice));
    
    // Zero out H and b blocks
    CUDA_CHECK(cudaMemset(d_H_blocks_, 0, num_source * 36 * sizeof(double)));
    CUDA_CHECK(cudaMemset(d_b_blocks_, 0, num_source * 6 * sizeof(double)));
    CUDA_CHECK(cudaMemset(d_errors_, 0, num_source * sizeof(double)));
    
    // Launch kernel
    const int THREADS_PER_BLOCK = 256;
    const int NUM_BLOCKS = (num_source + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    computeLinearizationKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
        d_source_points_, num_source,
        d_target_points_,
        d_correspondences_,
        d_mahalanobis_,
        d_transform_,
        d_H_blocks_,
        d_b_blocks_,
        d_errors_
    );
    
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Copy results back and reduce
    std::vector<double> h_H_blocks(num_source * 36);
    std::vector<double> h_b_blocks(num_source * 6);
    std::vector<double> h_errors(num_source);
    
    CUDA_CHECK(cudaMemcpy(h_H_blocks.data(), d_H_blocks_,
                         num_source * 36 * sizeof(double), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(h_b_blocks.data(), d_b_blocks_,
                         num_source * 6 * sizeof(double), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(h_errors.data(), d_errors_,
                         num_source * sizeof(double), cudaMemcpyDeviceToHost));
    
    // Reduce H matrices and b vectors
    H.setZero();
    b.setZero();
    double total_error = 0.0;
    
    for (int i = 0; i < num_source; i++) {
        // Add H contribution
        for (int row = 0; row < 6; row++) {
            for (int col = 0; col < 6; col++) {
                H(row, col) += h_H_blocks[i * 36 + row * 6 + col];
            }
        }
        
        // Add b contribution
        for (int row = 0; row < 6; row++) {
            b(row) += h_b_blocks[i * 6 + row];
        }
        
        // Add error contribution
        total_error += h_errors[i];
    }
    
    return total_error;
}

double CudaGICP::computeError(
    const float* source_points, int num_source,
    const float* target_points, int num_target,
    const int* correspondences,
    const GpuMatrix4* mahalanobis,
    const float* transform) {
    
    CUDA_CHECK(cudaMemcpy(d_correspondences_, correspondences,
                         num_source * sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_mahalanobis_, mahalanobis,
                         num_source * sizeof(GpuMatrix4), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_transform_, transform, 16 * sizeof(float),
                         cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemset(d_errors_, 0, num_source * sizeof(double)));
    
    const int THREADS_PER_BLOCK = 256;
    const int NUM_BLOCKS = (num_source + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    computeErrorKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
        d_source_points_, num_source,
        d_target_points_,
        d_correspondences_,
        d_mahalanobis_,
        d_transform_,
        d_errors_
    );
    
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    
    std::vector<double> h_errors(num_source);
    CUDA_CHECK(cudaMemcpy(h_errors.data(), d_errors_,
                         num_source * sizeof(double), cudaMemcpyDeviceToHost));
    
    double total_error = 0.0;
    for (int i = 0; i < num_source; i++) {
        total_error += h_errors[i];
    }
    
    return total_error;
}

void CudaGICP::clear() {
    if (d_source_points_) CUDA_CHECK(cudaFree(d_source_points_));
    if (d_target_points_) CUDA_CHECK(cudaFree(d_target_points_));
    if (d_source_covs_) CUDA_CHECK(cudaFree(d_source_covs_));
    if (d_target_covs_) CUDA_CHECK(cudaFree(d_target_covs_));
    if (d_correspondences_) CUDA_CHECK(cudaFree(d_correspondences_));
    if (d_sq_distances_) CUDA_CHECK(cudaFree(d_sq_distances_));
    if (d_mahalanobis_) CUDA_CHECK(cudaFree(d_mahalanobis_));
    if (d_transform_) CUDA_CHECK(cudaFree(d_transform_));
    if (d_H_blocks_) CUDA_CHECK(cudaFree(d_H_blocks_));
    if (d_b_blocks_) CUDA_CHECK(cudaFree(d_b_blocks_));
    if (d_errors_) CUDA_CHECK(cudaFree(d_errors_));
    
    d_source_points_ = nullptr;
    d_target_points_ = nullptr;
    d_source_covs_ = nullptr;
    d_target_covs_ = nullptr;
    d_correspondences_ = nullptr;
    d_sq_distances_ = nullptr;
    d_mahalanobis_ = nullptr;
    d_transform_ = nullptr;
    d_H_blocks_ = nullptr;
    d_b_blocks_ = nullptr;
    d_errors_ = nullptr;
    
    knn_search_.clear();
    
    max_source_points_ = 0;
    max_target_points_ = 0;
    initialized_ = false;
}

} // namespace cuda
} // namespace nano_gicp
