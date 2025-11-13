#include <cuda_runtime.h>
#include <cuda.h>
#include <stdio.h>

int main() {
    printf("Testing CUDA 13.0 availability...\n");
    
    // Try CUDA Driver API first
    CUresult cuResult = cuInit(0);
    if (cuResult != CUDA_SUCCESS) {
        const char* errStr;
        cuGetErrorString(cuResult, &errStr);
        printf("cuInit() failed: %d - %s\n", cuResult, errStr);
        printf("This indicates driver API incompatibility.\n");
    } else {
        printf("cuInit() succeeded\n");
    }
    
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    
    if (error != cudaSuccess) {
        printf("ERROR: cudaGetDeviceCount() returned %d: %s\n", 
               error, cudaGetErrorString(error));
        return 1;
    }
    
    if (deviceCount == 0) {
        printf("No CUDA devices found (deviceCount = 0)\n");
        return 1;
    }
    
    printf("SUCCESS: Found %d CUDA device(s)\n", deviceCount);
    
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    printf("Device 0: %s\n", prop.name);
    printf("Compute capability: %d.%d\n", prop.major, prop.minor);
    printf("Total global memory: %.2f GB\n", prop.totalGlobalMem / 1024.0 / 1024.0 / 1024.0);
    
    return 0;
}
