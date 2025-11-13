/***********************************************************
 * CUDA Integration Header for NanoGICP
 * Provides GPU-accelerated functions with CPU fallback
 ***********************************************************/

#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Forward declarations of CUDA classes
namespace nano_gicp {
namespace cuda {
    class CudaKNNSearch;
    class CudaCovarianceCalculator;
    class CudaPointCloudTransform;
    class CudaGICP;
    struct GpuMatrix4;
}
}

namespace nano_gicp {

// GPU acceleration configuration
struct GpuConfig {
    bool use_gpu = true;           // Enable/disable GPU acceleration
    bool verbose = false;          // Print GPU info
    int device_id = 0;             // CUDA device ID
    bool fallback_to_cpu = true;   // Automatically fall back to CPU if GPU fails
};

// GPU-accelerated helper class for NanoGICP
class GpuAccelerator {
public:
    GpuAccelerator();
    ~GpuAccelerator();
    
    // Initialize GPU (returns false if GPU is not available)
    bool initialize(const GpuConfig& config = GpuConfig());
    
    // Check if GPU is available and initialized
    bool isAvailable() const { return gpu_available_; }
    
    // Get GPU device properties
    std::string getDeviceInfo() const;
    
    // GPU-accelerated covariance calculation
    template<typename PointT>
    bool calculateCovariances(
        const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
        int k_correspondences,
        std::vector<Eigen::Matrix4d>& covariances,
        float& density);
    
    // GPU-accelerated point cloud transformation
    template<typename PointT>
    void transformPointCloud(
        const typename pcl::PointCloud<PointT>::ConstPtr& input,
        typename pcl::PointCloud<PointT>::Ptr& output,
        const Eigen::Matrix4f& transform);
    
    // Clear GPU memory
    void clear();
    
private:
    bool gpu_available_;
    GpuConfig config_;
    
    // CUDA objects (use pimpl to avoid exposing CUDA headers)
    std::shared_ptr<cuda::CudaKNNSearch> knn_search_;
    std::shared_ptr<cuda::CudaCovarianceCalculator> cov_calculator_;
    std::shared_ptr<cuda::CudaPointCloudTransform> transform_;
    std::shared_ptr<cuda::CudaGICP> gicp_;
    
    // Helper to convert PCL cloud to flat array
    template<typename PointT>
    void cloudToArray(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                     std::vector<float>& points);
    
    template<typename PointT>
    void arrayToCloud(const std::vector<float>& points,
                     typename pcl::PointCloud<PointT>::Ptr& cloud);
};

} // namespace nano_gicp
