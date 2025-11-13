/***********************************************************
 * CUDA-Accelerated KNN Search Implementation
 * Optimized for RTX 2060 (Turing architecture)
 ***********************************************************/

#include "nano_gicp/cuda/cuda_knn.cuh"
#include <algorithm>
#include <cstdio>
#include <cfloat>

namespace nano_gicp {
namespace cuda {

// Device function to insert element into sorted list
__device__ inline void insertSorted(float dist, int idx, float* dists, int* idxs, int k) {
    // Find insertion position
    int pos = k - 1;
    while (pos > 0 && dists[pos - 1] > dist) {
        pos--;
    }
    
    // Shift elements and insert
    if (pos < k) {
        for (int i = k - 1; i > pos; i--) {
            dists[i] = dists[i - 1];
            idxs[i] = idxs[i - 1];
        }
        dists[pos] = dist;
        idxs[pos] = idx;
    }
}

// Optimized KNN kernel using shared memory and warp-level primitives
__global__ void sharedMemKnnKernel(const GpuPoint* target_points, int num_targets,
                                   const GpuPoint* query_points, int num_queries,
                                   int k, int* indices, float* sq_distances) {
    const int query_idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (query_idx >= num_queries) return;
    
    // Shared memory for this block's results
    extern __shared__ float shared_mem[];
    float* thread_dists = shared_mem + threadIdx.x * k;
    int* thread_idxs = (int*)(shared_mem + blockDim.x * k) + threadIdx.x * k;
    
    // Initialize with max distance
    for (int i = 0; i < k; i++) {
        thread_dists[i] = FLT_MAX;
        thread_idxs[i] = -1;
    }
    
    GpuPoint query = query_points[query_idx];
    
    // Process target points in chunks for better cache usage
    const int CHUNK_SIZE = 128;
    for (int chunk_start = 0; chunk_start < num_targets; chunk_start += CHUNK_SIZE) {
        int chunk_end = min(chunk_start + CHUNK_SIZE, num_targets);
        
        // Process chunk
        for (int i = chunk_start; i < chunk_end; i++) {
            float dist = squaredDistance(query, target_points[i]);
            
            // Insert if it's among k nearest
            if (dist < thread_dists[k - 1]) {
                insertSorted(dist, i, thread_dists, thread_idxs, k);
            }
        }
    }
    
    // Write results to global memory
    int out_offset = query_idx * k;
    for (int i = 0; i < k; i++) {
        sq_distances[out_offset + i] = thread_dists[i];
        indices[out_offset + i] = thread_idxs[i];
    }
}

// Brute force KNN kernel (fallback for large k)
__global__ void bruteForceKnnKernel(const GpuPoint* target_points, int num_targets,
                                    const GpuPoint* query_points, int num_queries,
                                    int k, int* indices, float* sq_distances) {
    const int query_idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (query_idx >= num_queries) return;
    
    // Per-thread storage for k nearest neighbors
    float local_dists[32]; // Adjust based on typical k value
    int local_idxs[32];
    
    int actual_k = min(k, 32);
    
    // Initialize
    for (int i = 0; i < actual_k; i++) {
        local_dists[i] = FLT_MAX;
        local_idxs[i] = -1;
    }
    
    GpuPoint query = query_points[query_idx];
    
    // Find k nearest neighbors
    for (int i = 0; i < num_targets; i++) {
        float dist = squaredDistance(query, target_points[i]);
        
        if (dist < local_dists[actual_k - 1]) {
            insertSorted(dist, i, local_dists, local_idxs, actual_k);
        }
    }
    
    // Write results
    int out_offset = query_idx * k;
    for (int i = 0; i < actual_k; i++) {
        sq_distances[out_offset + i] = local_dists[i];
        indices[out_offset + i] = local_idxs[i];
    }
}

// CudaKNNSearch implementation
CudaKNNSearch::CudaKNNSearch() 
    : d_target_points_(nullptr)
    , d_query_points_(nullptr)
    , d_indices_(nullptr)
    , d_distances_(nullptr)
    , num_target_points_(0)
    , max_query_points_(0)
    , max_k_(0)
    , initialized_(false) {
}

CudaKNNSearch::~CudaKNNSearch() {
    clear();
}

void CudaKNNSearch::setTargetCloud(const float* points, int num_points) {
    // Free previous target cloud
    if (d_target_points_) {
        CUDA_CHECK(cudaFree(d_target_points_));
    }
    
    num_target_points_ = num_points;
    
    // Allocate and copy target points
    size_t bytes = num_points * sizeof(GpuPoint);
    CUDA_CHECK(cudaMalloc(&d_target_points_, bytes));
    
    // Convert and copy points (assuming input is [x,y,z,x,y,z,...])
    std::vector<GpuPoint> h_points(num_points);
    for (int i = 0; i < num_points; i++) {
        h_points[i].x = points[i * 3 + 0];
        h_points[i].y = points[i * 3 + 1];
        h_points[i].z = points[i * 3 + 2];
        h_points[i].w = 1.0f;
    }
    
    CUDA_CHECK(cudaMemcpy(d_target_points_, h_points.data(), bytes, cudaMemcpyHostToDevice));
}

void CudaKNNSearch::allocateMemory(int max_queries, int k) {
    if (initialized_ && max_queries <= max_query_points_ && k <= max_k_) {
        return; // Already allocated
    }
    
    // Free existing memory
    if (d_query_points_) CUDA_CHECK(cudaFree(d_query_points_));
    if (d_indices_) CUDA_CHECK(cudaFree(d_indices_));
    if (d_distances_) CUDA_CHECK(cudaFree(d_distances_));
    
    max_query_points_ = max_queries;
    max_k_ = k;
    
    // Allocate memory
    CUDA_CHECK(cudaMalloc(&d_query_points_, max_queries * sizeof(GpuPoint)));
    CUDA_CHECK(cudaMalloc(&d_indices_, max_queries * k * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_distances_, max_queries * k * sizeof(float)));
    
    initialized_ = true;
}

void CudaKNNSearch::batchKnnSearch(const float* query_points, int num_queries, int k,
                                  int* indices, float* sq_distances) {
    if (num_target_points_ == 0) {
        fprintf(stderr, "Error: Target cloud not set\n");
        return;
    }
    
    // Allocate memory if needed
    allocateMemory(num_queries, k);
    
    // Convert and copy query points to GPU
    std::vector<GpuPoint> h_queries(num_queries);
    for (int i = 0; i < num_queries; i++) {
        h_queries[i].x = query_points[i * 3 + 0];
        h_queries[i].y = query_points[i * 3 + 1];
        h_queries[i].z = query_points[i * 3 + 2];
        h_queries[i].w = 1.0f;
    }
    
    CUDA_CHECK(cudaMemcpy(d_query_points_, h_queries.data(), 
                         num_queries * sizeof(GpuPoint), cudaMemcpyHostToDevice));
    
    // Launch kernel
    const int THREADS_PER_BLOCK = 256; // Optimized for RTX 2060
    const int NUM_BLOCKS = (num_queries + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    if (k <= 20) {
        // Use optimized shared memory kernel for small k
        size_t shared_mem_size = THREADS_PER_BLOCK * k * (sizeof(float) + sizeof(int));
        sharedMemKnnKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK, shared_mem_size>>>(
            d_target_points_, num_target_points_,
            d_query_points_, num_queries,
            k, d_indices_, d_distances_
        );
    } else {
        // Use brute force kernel for larger k
        bruteForceKnnKernel<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(
            d_target_points_, num_target_points_,
            d_query_points_, num_queries,
            k, d_indices_, d_distances_
        );
    }
    
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Copy results back to host
    CUDA_CHECK(cudaMemcpy(indices, d_indices_, num_queries * k * sizeof(int), 
                         cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(sq_distances, d_distances_, num_queries * k * sizeof(float), 
                         cudaMemcpyDeviceToHost));
}

void CudaKNNSearch::clear() {
    if (d_target_points_) {
        CUDA_CHECK(cudaFree(d_target_points_));
        d_target_points_ = nullptr;
    }
    if (d_query_points_) {
        CUDA_CHECK(cudaFree(d_query_points_));
        d_query_points_ = nullptr;
    }
    if (d_indices_) {
        CUDA_CHECK(cudaFree(d_indices_));
        d_indices_ = nullptr;
    }
    if (d_distances_) {
        CUDA_CHECK(cudaFree(d_distances_));
        d_distances_ = nullptr;
    }
    
    num_target_points_ = 0;
    max_query_points_ = 0;
    initialized_ = false;
}

} // namespace cuda
} // namespace nano_gicp
