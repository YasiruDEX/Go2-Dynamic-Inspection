/***********************************************************
 * CUDA Integration Implementation for NanoGICP
 ***********************************************************/

#include "nano_gicp/gpu_accelerator.h"
#include "nano_gicp/cuda/cuda_knn.cuh"
#include "nano_gicp/cuda/cuda_covariance.cuh"
#include "nano_gicp/cuda/cuda_transform.cuh"
#include "nano_gicp/cuda/cuda_gicp.cuh"
#include <cuda_runtime.h>
#include <iostream>

namespace nano_gicp {

GpuAccelerator::GpuAccelerator()
    : gpu_available_(false)
    , knn_search_(nullptr)
    , cov_calculator_(nullptr)
    , transform_(nullptr)
    , gicp_(nullptr) {
}

GpuAccelerator::~GpuAccelerator() {
    clear();
}

bool GpuAccelerator::initialize(const GpuConfig& config) {
    config_ = config;
    
    if (!config_.use_gpu) {
        if (config_.verbose) {
            std::cout << "[GPU Accelerator] GPU disabled by configuration" << std::endl;
        }
        return false;
    }
    
    // Check CUDA availability
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    
    if (error != cudaSuccess || deviceCount == 0) {
        if (config_.verbose) {
            std::cerr << "[GPU Accelerator] No CUDA-capable device found" << std::endl;
        }
        gpu_available_ = false;
        return false;
    }
    
    // Set device
    error = cudaSetDevice(config_.device_id);
    if (error != cudaSuccess) {
        if (config_.verbose) {
            std::cerr << "[GPU Accelerator] Failed to set CUDA device " 
                     << config_.device_id << std::endl;
        }
        gpu_available_ = false;
        return false;
    }
    
    // Get device properties
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, config_.device_id);
    
    if (config_.verbose) {
        std::cout << "[GPU Accelerator] Initialized successfully" << std::endl;
        std::cout << "  Device: " << prop.name << std::endl;
        std::cout << "  Compute Capability: " << prop.major << "." << prop.minor << std::endl;
        std::cout << "  Global Memory: " << prop.totalGlobalMem / (1024*1024) << " MB" << std::endl;
        std::cout << "  Multiprocessors: " << prop.multiProcessorCount << std::endl;
        std::cout << "  Max Threads per Block: " << prop.maxThreadsPerBlock << std::endl;
    }
    
    // Initialize CUDA objects
    try {
        knn_search_ = std::make_shared<cuda::CudaKNNSearch>();
        cov_calculator_ = std::make_shared<cuda::CudaCovarianceCalculator>();
        transform_ = std::make_shared<cuda::CudaPointCloudTransform>();
        gicp_ = std::make_shared<cuda::CudaGICP>();
        
        gpu_available_ = true;
        return true;
    } catch (const std::exception& e) {
        if (config_.verbose) {
            std::cerr << "[GPU Accelerator] Failed to initialize CUDA objects: " 
                     << e.what() << std::endl;
        }
        gpu_available_ = false;
        return false;
    }
}

std::string GpuAccelerator::getDeviceInfo() const {
    if (!gpu_available_) {
        return "GPU not available";
    }
    
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, config_.device_id);
    
    std::string info = "GPU: ";
    info += prop.name;
    info += " (CC: " + std::to_string(prop.major) + "." + std::to_string(prop.minor) + ")";
    info += ", Memory: " + std::to_string(prop.totalGlobalMem / (1024*1024)) + " MB";
    
    return info;
}

template<typename PointT>
void GpuAccelerator::cloudToArray(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                                  std::vector<float>& points) {
    points.resize(cloud->size() * 3);
    for (size_t i = 0; i < cloud->size(); i++) {
        points[i * 3 + 0] = cloud->points[i].x;
        points[i * 3 + 1] = cloud->points[i].y;
        points[i * 3 + 2] = cloud->points[i].z;
    }
}

template<typename PointT>
void GpuAccelerator::arrayToCloud(const std::vector<float>& points,
                                  typename pcl::PointCloud<PointT>::Ptr& cloud) {
    size_t num_points = points.size() / 3;
    cloud->resize(num_points);
    for (size_t i = 0; i < num_points; i++) {
        cloud->points[i].x = points[i * 3 + 0];
        cloud->points[i].y = points[i * 3 + 1];
        cloud->points[i].z = points[i * 3 + 2];
    }
}

template<typename PointT>
bool GpuAccelerator::calculateCovariances(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    int k_correspondences,
    std::vector<Eigen::Matrix4d>& covariances,
    float& density) {
    
    if (!gpu_available_ || !cov_calculator_ || !knn_search_) {
        return false;
    }
    
    try {
        // Convert cloud to flat array
        std::vector<float> points;
        cloudToArray<PointT>(cloud, points);
        
        // Set cloud as both source and target for self-KNN
        knn_search_->setTargetCloud(points.data(), cloud->size());
        
        // Calculate covariances
        return cov_calculator_->calculateCovariances(
            points.data(), cloud->size(),
            *knn_search_, k_correspondences,
            covariances, density
        );
    } catch (const std::exception& e) {
        if (config_.verbose) {
            std::cerr << "[GPU Accelerator] Covariance calculation failed: " 
                     << e.what() << std::endl;
        }
        return false;
    }
}

template<typename PointT>
void GpuAccelerator::transformPointCloud(
    const typename pcl::PointCloud<PointT>::ConstPtr& input,
    typename pcl::PointCloud<PointT>::Ptr& output,
    const Eigen::Matrix4f& transform) {
    
    if (!gpu_available_ || !transform_) {
        // Fallback: use PCL transformation
        pcl::transformPointCloud(*input, *output, transform);
        return;
    }
    
    try {
        // Convert to flat array
        std::vector<float> points;
        cloudToArray<PointT>(input, points);
        
        // Convert transform to row-major array
        float transform_array[16];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transform_array[i * 4 + j] = transform(i, j);
            }
        }
        
        // Transform on GPU
        transform_->transformPointCloud(points.data(), input->size(), transform_array);
        
        // Convert back
        arrayToCloud<PointT>(points, output);
    } catch (const std::exception& e) {
        if (config_.verbose) {
            std::cerr << "[GPU Accelerator] Transform failed: " << e.what() << std::endl;
        }
        // Fallback
        pcl::transformPointCloud(*input, *output, transform);
    }
}

void GpuAccelerator::clear() {
    if (knn_search_) knn_search_->clear();
    if (cov_calculator_) cov_calculator_->clear();
    if (transform_) transform_->clear();
    if (gicp_) gicp_->clear();
}

// Explicit template instantiations for common point types
template void GpuAccelerator::cloudToArray<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&, std::vector<float>&);
template void GpuAccelerator::cloudToArray<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&, std::vector<float>&);

template void GpuAccelerator::arrayToCloud<pcl::PointXYZ>(
    const std::vector<float>&, pcl::PointCloud<pcl::PointXYZ>::Ptr&);
template void GpuAccelerator::arrayToCloud<pcl::PointXYZI>(
    const std::vector<float>&, pcl::PointCloud<pcl::PointXYZI>::Ptr&);

template bool GpuAccelerator::calculateCovariances<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&, int,
    std::vector<Eigen::Matrix4d>&, float&);
template bool GpuAccelerator::calculateCovariances<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&, int,
    std::vector<Eigen::Matrix4d>&, float&);

template void GpuAccelerator::transformPointCloud<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&,
    pcl::PointCloud<pcl::PointXYZ>::Ptr&, const Eigen::Matrix4f&);
template void GpuAccelerator::transformPointCloud<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,
    pcl::PointCloud<pcl::PointXYZI>::Ptr&, const Eigen::Matrix4f&);

} // namespace nano_gicp
