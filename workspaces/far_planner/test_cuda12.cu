#include <stdio.h>
#include <cuda_runtime.h>

int main() {
    printf("Testing CUDA 12.6...\n");
    
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    
    printf("cudaGetDeviceCount returned: %d (%s)\n", error, cudaGetErrorString(error));
    printf("Device count: %d\n", deviceCount);
    
    if (error == cudaSuccess && deviceCount > 0) {
        cudaDeviceProp prop;
        error = cudaGetDeviceProperties(&prop, 0);
        printf("cudaGetDeviceProperties returned: %d (%s)\n", error, cudaGetErrorString(error));
        if (error == cudaSuccess) {
            printf("Device 0: %s\n", prop.name);
            printf("Compute Capability: %d.%d\n", prop.major, prop.minor);
        }
    }
    
    return 0;
}
