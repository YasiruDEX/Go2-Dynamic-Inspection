/***********************************************************
 * CUDA-Accelerated KNN Search for DLIO
 * Optimized for RTX 2060
 ***********************************************************/

#pragma once

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <vector>
#include <memory>

namespace nano_gicp {
namespace cuda {

// GPU point structure (aligned for coalesced memory access)
struct __align__(16) GpuPoint {
    float x, y, z, w;
    
    __device__ __host__ GpuPoint() : x(0), y(0), z(0), w(1) {}
    __device__ __host__ GpuPoint(float _x, float _y, float _z) : x(_x), y(_y), z(_z), w(1) {}
};

// CUDA error checking macro
#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__, \
                    cudaGetErrorString(error)); \
            exit(EXIT_FAILURE); \
        } \
    } while(0)

class CudaKNNSearch {
public:
    CudaKNNSearch();
    ~CudaKNNSearch();
    
    // Set target point cloud (reference cloud for KNN queries)
    void setTargetCloud(const float* points, int num_points);
    
    // Perform batch KNN search
    // query_points: source points to find neighbors for
    // num_queries: number of query points
    // k: number of nearest neighbors
    // indices: output array of size [num_queries * k]
    // distances: output squared distances [num_queries * k]
    void batchKnnSearch(const float* query_points, int num_queries, int k,
                       int* indices, float* sq_distances);
    
    // Clear GPU memory
    void clear();
    
private:
    // GPU memory pointers
    GpuPoint* d_target_points_;
    GpuPoint* d_query_points_;
    int* d_indices_;
    float* d_distances_;
    
    int num_target_points_;
    int max_query_points_;
    int max_k_;
    
    bool initialized_;
    
    // Allocate GPU memory
    void allocateMemory(int max_queries, int k);
};

// CUDA kernel declarations
__global__ void bruteForceKnnKernel(const GpuPoint* target_points, int num_targets,
                                    const GpuPoint* query_points, int num_queries,
                                    int k, int* indices, float* sq_distances);

// Optimized shared memory version for small k
__global__ void sharedMemKnnKernel(const GpuPoint* target_points, int num_targets,
                                   const GpuPoint* query_points, int num_queries,
                                   int k, int* indices, float* sq_distances);

// Helper device functions
__device__ inline float squaredDistance(const GpuPoint& a, const GpuPoint& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx*dx + dy*dy + dz*dz;
}

// Warp-level primitives for parallel reduction
__device__ inline void insertSorted(float dist, int idx, float* dists, int* idxs, int k);

} // namespace cuda
} // namespace nano_gicp
