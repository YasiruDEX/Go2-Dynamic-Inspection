#!/bin/bash

# DLIO GPU-Accelerated Build Script
# For RTX 2060 and compatible NVIDIA GPUs

set -e  # Exit on error

echo "======================================"
echo "DLIO GPU-Accelerated Build Script"
echo "======================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check CUDA installation
echo "Checking CUDA installation..."
if ! command -v nvcc &> /dev/null; then
    echo -e "${RED}Error: CUDA compiler (nvcc) not found!${NC}"
    echo "Please install CUDA Toolkit: sudo apt-get install nvidia-cuda-toolkit"
    exit 1
fi

CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $5}' | cut -d',' -f1)
echo -e "${GREEN}✓ CUDA found: version $CUDA_VERSION${NC}"

# Check GPU
echo ""
echo "Checking GPU..."
if ! command -v nvidia-smi &> /dev/null; then
    echo -e "${YELLOW}Warning: nvidia-smi not found. Cannot verify GPU.${NC}"
else
    GPU_INFO=$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | head -n 1)
    echo -e "${GREEN}✓ GPU detected: $GPU_INFO${NC}"
fi

# Detect GPU compute capability
echo ""
echo "Detecting GPU compute capability..."
if command -v nvidia-smi &> /dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n 1)
    
    # Map common GPUs to compute capability
    if [[ "$GPU_NAME" == *"RTX 2060"* ]] || [[ "$GPU_NAME" == *"RTX 2070"* ]] || [[ "$GPU_NAME" == *"RTX 2080"* ]]; then
        COMPUTE_CAP="75"
        echo -e "${GREEN}✓ Detected RTX 20-series: Using compute capability 7.5${NC}"
    elif [[ "$GPU_NAME" == *"RTX 3060"* ]] || [[ "$GPU_NAME" == *"RTX 3070"* ]] || [[ "$GPU_NAME" == *"RTX 3080"* ]] || [[ "$GPU_NAME" == *"RTX 3090"* ]]; then
        COMPUTE_CAP="86"
        echo -e "${YELLOW}Detected RTX 30-series: Using compute capability 8.6${NC}"
        echo -e "${YELLOW}Note: You may need to update CMakeLists.txt for optimal performance${NC}"
    elif [[ "$GPU_NAME" == *"GTX 1080"* ]] || [[ "$GPU_NAME" == *"GTX 1070"* ]]; then
        COMPUTE_CAP="61"
        echo -e "${YELLOW}Detected GTX 10-series: Using compute capability 6.1${NC}"
    else
        COMPUTE_CAP="75"
        echo -e "${YELLOW}Unknown GPU, defaulting to compute capability 7.5 (RTX 2060)${NC}"
    fi
else
    COMPUTE_CAP="75"
    echo -e "${YELLOW}Cannot detect GPU, defaulting to compute capability 7.5 (RTX 2060)${NC}"
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if we're in a ROS workspace
if [ ! -f "CMakeLists.txt" ]; then
    echo -e "${RED}Error: CMakeLists.txt not found!${NC}"
    echo "Please run this script from the DLIO source directory."
    exit 1
fi

# Clean previous build (optional)
if [ "$1" == "clean" ]; then
    echo ""
    echo "Cleaning previous build..."
    rm -rf build install log
    echo -e "${GREEN}✓ Clean complete${NC}"
fi

# Build with colcon
echo ""
echo "Building with colcon..."
echo "This may take a few minutes on first build..."
echo ""

# Set build options
export CMAKE_BUILD_TYPE=Release
export CMAKE_CUDA_ARCHITECTURES=$COMPUTE_CAP

# Build
colcon build \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES=$COMPUTE_CAP \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --parallel-workers $(nproc)

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}======================================"
    echo "✓ Build completed successfully!"
    echo "======================================${NC}"
    echo ""
    echo "GPU-accelerated DLIO is ready to use."
    echo ""
    echo "To run:"
    echo "  source install/setup.bash"
    echo "  ros2 launch direct_lidar_inertial_odometry dlio.launch.py"
    echo ""
    echo "To monitor GPU usage:"
    echo "  watch -n 0.5 nvidia-smi"
    echo ""
    echo "For more information, see GPU_ACCELERATION.md"
else
    echo ""
    echo -e "${RED}======================================"
    echo "✗ Build failed!"
    echo "======================================${NC}"
    echo ""
    echo "Common fixes:"
    echo "  1. Check CUDA installation: nvcc --version"
    echo "  2. Verify GPU driver: nvidia-smi"
    echo "  3. Check build log above for specific errors"
    echo "  4. See GPU_ACCELERATION.md for troubleshooting"
    exit 1
fi
