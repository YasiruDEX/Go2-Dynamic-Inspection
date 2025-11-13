/***********************************************************
 * CUDA-Accelerated Covariance Matrix Computation Implementation
 * Optimized for RTX 2060 (Turing architecture)
 ***********************************************************/

#include "nano_gicp/cuda/cuda_covariance.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cfloat>
#include <algorithm>

namespace nano_gicp {
namespace cuda {

// Simplified SVD for 3x3 symmetric matrices (Jacobi method)
__device__ void computeJacobiSVD3x3(const float* A, float* U, float* S, float* V) {
    // Initialize U and V as identity
    U[0] = U[4] = U[8] = 1.0f;
    U[1] = U[2] = U[3] = U[5] = U[6] = U[7] = 0.0f;
    
    V[0] = V[4] = V[8] = 1.0f;
    V[1] = V[2] = V[3] = V[5] = V[6] = V[7] = 0.0f;
    
    // Copy A to working matrix
    float M[9];
    for (int i = 0; i < 9; i++) M[i] = A[i];
    
    // Jacobi iteration (simplified, few iterations)
    const int MAX_ITER = 10;
    const float EPSILON = 1e-6f;
    
    for (int iter = 0; iter < MAX_ITER; iter++) {
        // Find largest off-diagonal element
        float max_val = 0.0f;
        int p = 0, q = 1;
        
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                float val = fabsf(M[i * 3 + j]);
                if (val > max_val) {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }
        
        if (max_val < EPSILON) break;
        
        // Compute Jacobi rotation
        float theta = 0.5f * atan2f(2.0f * M[p * 3 + q], M[q * 3 + q] - M[p * 3 + p]);
        float c = cosf(theta);
        float s = sinf(theta);
        
        // Apply rotation to M
        float M_pp = M[p * 3 + p];
        float M_qq = M[q * 3 + q];
        float M_pq = M[p * 3 + q];
        
        M[p * 3 + p] = c * c * M_pp - 2.0f * s * c * M_pq + s * s * M_qq;
        M[q * 3 + q] = s * s * M_pp + 2.0f * s * c * M_pq + c * c * M_qq;
        M[p * 3 + q] = M[q * 3 + p] = 0.0f;
        
        // Update other elements
        for (int i = 0; i < 3; i++) {
            if (i != p && i != q) {
                float M_ip = M[i * 3 + p];
                float M_iq = M[i * 3 + q];
                M[i * 3 + p] = M[p * 3 + i] = c * M_ip - s * M_iq;
                M[i * 3 + q] = M[q * 3 + i] = s * M_ip + c * M_iq;
            }
        }
        
        // Update V (eigenvectors)
        for (int i = 0; i < 3; i++) {
            float V_ip = V[i * 3 + p];
            float V_iq = V[i * 3 + q];
            V[i * 3 + p] = c * V_ip - s * V_iq;
            V[i * 3 + q] = s * V_ip + c * V_iq;
        }
    }
    
    // Extract eigenvalues (diagonal of M)
    S[0] = M[0];
    S[1] = M[4];
    S[2] = M[8];
    
    // Sort eigenvalues in descending order
    if (S[0] < S[1]) {
        float tmp = S[0]; S[0] = S[1]; S[1] = tmp;
        for (int i = 0; i < 3; i++) {
            tmp = V[i * 3 + 0]; V[i * 3 + 0] = V[i * 3 + 1]; V[i * 3 + 1] = tmp;
        }
    }
    if (S[1] < S[2]) {
        float tmp = S[1]; S[1] = S[2]; S[2] = tmp;
        for (int i = 0; i < 3; i++) {
            tmp = V[i * 3 + 1]; V[i * 3 + 1] = V[i * 3 + 2]; V[i * 3 + 2] = tmp;
        }
    }
    if (S[0] < S[1]) {
        float tmp = S[0]; S[0] = S[1]; S[1] = tmp;
        for (int i = 0; i < 3; i++) {
            tmp = V[i * 3 + 0]; V[i * 3 + 0] = V[i * 3 + 1]; V[i * 3 + 1] = tmp;
        }
    }
    
    // U = V for symmetric matrices
    for (int i = 0; i < 9; i++) U[i] = V[i];
}

__device__ void matrixMultiply3x3(const float* A, const float* B, float* C) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += A[i * 3 + k] * B[k * 3 + j];
            }
            C[i * 3 + j] = sum;
        }
    }
}

// Main kernel to compute covariances
__global__ void computeCovariancesKernel(
    const GpuPoint* points, int num_points,
    const int* knn_indices, const float* knn_distances,
    int k, GpuMatrix4* covariances,
    float* density_contributions) {
    
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    
    // Compute mean of neighbors
    float mean[3] = {0.0f, 0.0f, 0.0f};
    int valid_neighbors = 0;
    
    int knn_offset = idx * k;
    for (int i = 0; i < k; i++) {
        int neighbor_idx = knn_indices[knn_offset + i];
        if (neighbor_idx >= 0) {
            mean[0] += points[neighbor_idx].x;
            mean[1] += points[neighbor_idx].y;
            mean[2] += points[neighbor_idx].z;
            valid_neighbors++;
        }
    }
    
    if (valid_neighbors > 0) {
        mean[0] /= valid_neighbors;
        mean[1] /= valid_neighbors;
        mean[2] /= valid_neighbors;
    }
    
    // Compute covariance matrix (3x3, upper triangle)
    float cov[9] = {0.0f};
    
    for (int i = 0; i < k; i++) {
        int neighbor_idx = knn_indices[knn_offset + i];
        if (neighbor_idx >= 0) {
            float dx = points[neighbor_idx].x - mean[0];
            float dy = points[neighbor_idx].y - mean[1];
            float dz = points[neighbor_idx].z - mean[2];
            
            cov[0] += dx * dx; // xx
            cov[1] += dx * dy; // xy
            cov[2] += dx * dz; // xz
            cov[4] += dy * dy; // yy
            cov[5] += dy * dz; // yz
            cov[8] += dz * dz; // zz
        }
    }
    
    // Normalize
    if (valid_neighbors > 1) {
        float scale = 1.0f / valid_neighbors;
        for (int i = 0; i < 9; i++) {
            cov[i] *= scale;
        }
    }
    
    // Make symmetric
    cov[3] = cov[1];
    cov[6] = cov[2];
    cov[7] = cov[5];
    
    // Compute density contribution (average squared distance, excluding first neighbor which is the point itself)
    float density_sum = 0.0f;
    int density_count = 0;
    for (int i = 1; i < k; i++) { // Start from 1 to skip self
        if (knn_indices[knn_offset + i] >= 0) {
            density_sum += knn_distances[knn_offset + i];
            density_count++;
        }
    }
    
    if (density_count > 0) {
        const int normalization = ((k - 2) * (1 + k)) / 2;
        density_contributions[idx] = density_sum / (density_count > 0 ? normalization : 1);
    } else {
        density_contributions[idx] = 0.0f;
    }
    
    // Apply plane regularization using simplified SVD
    float U[9], S[3], V[9];
    computeJacobiSVD3x3(cov, U, S, V);
    
    // Plane regularization: set smallest eigenvalue to a small value
    S[0] = fmaxf(S[0], 1e-3f);
    S[1] = fmaxf(S[1], 1e-3f);
    S[2] = 1e-3f; // Force planar structure
    
    // Reconstruct covariance: cov = V * diag(S) * V^T
    float S_diag[9] = {0.0f};
    S_diag[0] = S[0];
    S_diag[4] = S[1];
    S_diag[8] = S[2];
    
    float VS[9], regularized_cov[9];
    matrixMultiply3x3(V, S_diag, VS);
    
    // V^T is V transposed
    float VT[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            VT[i * 3 + j] = V[j * 3 + i];
        }
    }
    
    matrixMultiply3x3(VS, VT, regularized_cov);
    
    // Store as 4x4 matrix (pad with zeros and 1 at [3,3])
    GpuMatrix4& output = covariances[idx];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            output.data[i * 4 + j] = regularized_cov[i * 3 + j];
        }
        output.data[i * 4 + 3] = 0.0f;
    }
    output.data[12] = output.data[13] = output.data[14] = 0.0f;
    output.data[15] = 0.0f;
}

// CudaCovarianceCalculator implementation
CudaCovarianceCalculator::CudaCovarianceCalculator()
    : d_points_(nullptr)
    , d_knn_indices_(nullptr)
    , d_knn_distances_(nullptr)
    , d_covariances_(nullptr)
    , d_density_contributions_(nullptr)
    , max_points_(0)
    , max_k_(0)
    , initialized_(false) {
}

CudaCovarianceCalculator::~CudaCovarianceCalculator() {
    clear();
}

void CudaCovarianceCalculator::allocateMemory(int max_points, int k) {
    if (initialized_ && max_points <= max_points_ && k <= max_k_) {
        return;
    }
    
    clear();
    
    max_points_ = max_points;
    max_k_ = k;
    
    CUDA_CHECK(cudaMalloc(&d_points_, max_points * sizeof(GpuPoint)));
    CUDA_CHECK(cudaMalloc(&d_knn_indices_, max_points * k * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_knn_distances_, max_points * k * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_covariances_, max_points * sizeof(GpuMatrix4)));
    CUDA_CHECK(cudaMalloc(&d_density_contributions_, max_points * sizeof(float)));
    
    initialized_ = true;
}

bool CudaCovarianceCalculator::calculateCovariances(
    const float* points, int num_points,
    CudaKNNSearch& knn_search, int k,
    std::vector<Eigen::Matrix4d>& covariances,
    float& density) {
    
    allocateMemory(num_points, k);
    
    // Perform KNN search
    knn_search.batchKnnSearch(points, num_points, k, d_knn_indices_, d_knn_distances_);
    
    // Copy points to GPU
    std::vector<GpuPoint> h_points(num_points);
    for (int i = 0; i < num_points; i++) {
        h_points[i].x = points[i * 3 + 0];
        h_points[i].y = points[i * 3 + 1];
        h_points[i].z = points[i * 3 + 2];
        h_points[i].w = 1.0f;
    }
    CUDA_CHECK(cudaMemcpy(d_points_, h_points.data(), 
                         num_points * sizeof(GpuPoint), cudaMemcpyHostToDevice));
    
    // Launch covariance computation kernel
    const int THREADS_PER_BLOCK = 256;
    const int NUM_BLOCKS = (num_points + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    computeCovariancesKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
        d_points_, num_points,
        d_knn_indices_, d_knn_distances_,
        k, d_covariances_, d_density_contributions_
    );
    
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Copy results back to host
    std::vector<GpuMatrix4> h_covariances(num_points);
    CUDA_CHECK(cudaMemcpy(h_covariances.data(), d_covariances_,
                         num_points * sizeof(GpuMatrix4), cudaMemcpyDeviceToHost));
    
    std::vector<float> h_density_contributions(num_points);
    CUDA_CHECK(cudaMemcpy(h_density_contributions.data(), d_density_contributions_,
                         num_points * sizeof(float), cudaMemcpyDeviceToHost));
    
    // Convert to Eigen format
    covariances.resize(num_points);
    for (int i = 0; i < num_points; i++) {
        Eigen::Matrix4d& cov = covariances[i];
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                cov(row, col) = h_covariances[i].data[row * 4 + col];
            }
        }
    }
    
    // Compute average density
    density = 0.0f;
    for (int i = 0; i < num_points; i++) {
        density += h_density_contributions[i];
    }
    density /= num_points;
    
    return true;
}

void CudaCovarianceCalculator::clear() {
    if (d_points_) {
        CUDA_CHECK(cudaFree(d_points_));
        d_points_ = nullptr;
    }
    if (d_knn_indices_) {
        CUDA_CHECK(cudaFree(d_knn_indices_));
        d_knn_indices_ = nullptr;
    }
    if (d_knn_distances_) {
        CUDA_CHECK(cudaFree(d_knn_distances_));
        d_knn_distances_ = nullptr;
    }
    if (d_covariances_) {
        CUDA_CHECK(cudaFree(d_covariances_));
        d_covariances_ = nullptr;
    }
    if (d_density_contributions_) {
        CUDA_CHECK(cudaFree(d_density_contributions_));
        d_density_contributions_ = nullptr;
    }
    
    max_points_ = 0;
    initialized_ = false;
}

} // namespace cuda
} // namespace nano_gicp
