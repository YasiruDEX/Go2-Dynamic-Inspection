#!/bin/bash

# GPU Setup and Verification Script for FAR Planner
# This script checks CUDA installation and GPU availability

echo "============================================"
echo "FAR Planner GPU Acceleration Setup Check"
echo "============================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check 1: NVIDIA Driver
echo -n "Checking NVIDIA driver... "
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}OK${NC}"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
else
    echo -e "${RED}FAILED${NC}"
    echo "  NVIDIA driver not found. Install with:"
    echo "  sudo apt-get install nvidia-driver-XXX"
    exit 1
fi

echo ""

# Check 2: CUDA Toolkit
echo -n "Checking CUDA Toolkit... "
if command -v nvcc &> /dev/null; then
    echo -e "${GREEN}OK${NC}"
    nvcc --version | grep "release"
else
    echo -e "${RED}FAILED${NC}"
    echo "  CUDA Toolkit not found. Install with:"
    echo "  sudo apt-get install nvidia-cuda-toolkit"
    echo "  or download from https://developer.nvidia.com/cuda-downloads"
    exit 1
fi

echo ""

# Check 3: GPU Compute Capability
echo "GPU Information:"
if command -v nvidia-smi &> /dev/null; then
    COMPUTE_CAP=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -n 1)
    echo "  Compute Capability: $COMPUTE_CAP"
    
    # Convert to number for comparison
    COMPUTE_NUM=$(echo "$COMPUTE_CAP" | tr -d '.')
    
    if [ "$COMPUTE_NUM" -ge 61 ]; then
        echo -e "  ${GREEN}Compatible with FAR Planner GPU acceleration${NC}"
    else
        echo -e "  ${YELLOW}Warning: Old GPU. Minimum recommended is 6.1 (GTX 1060+)${NC}"
    fi
fi

echo ""

# Check 4: CUDA Path Configuration
echo -n "Checking CUDA environment variables... "
if [ -n "$CUDA_HOME" ] || [ -n "$CUDA_PATH" ]; then
    echo -e "${GREEN}OK${NC}"
    echo "  CUDA_HOME: $CUDA_HOME"
    echo "  CUDA_PATH: $CUDA_PATH"
else
    echo -e "${YELLOW}WARNING${NC}"
    echo "  CUDA environment variables not set. Add to ~/.bashrc:"
    echo "  export CUDA_HOME=/usr/local/cuda"
    echo "  export PATH=\$CUDA_HOME/bin:\$PATH"
    echo "  export LD_LIBRARY_PATH=\$CUDA_HOME/lib64:\$LD_LIBRARY_PATH"
fi

echo ""

# Check 5: CMake CUDA Support
echo -n "Checking CMake CUDA support... "
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n 1 | awk '{print $3}')
    CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
    CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d. -f2)
    
    if [ "$CMAKE_MAJOR" -gt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -ge 8 ]); then
        echo -e "${GREEN}OK${NC} (version $CMAKE_VERSION)"
    else
        echo -e "${YELLOW}WARNING${NC} (version $CMAKE_VERSION)"
        echo "  CMake 3.8+ recommended for CUDA support"
    fi
else
    echo -e "${RED}FAILED${NC}"
    echo "  CMake not found"
fi

echo ""

# Check 6: Test CUDA Compilation
echo "Testing CUDA compilation..."
TEST_DIR=$(mktemp -d)
cat > $TEST_DIR/test.cu << 'EOF'
#include <stdio.h>
__global__ void test_kernel() {
    printf("CUDA is working!\n");
}
int main() {
    test_kernel<<<1, 1>>>();
    cudaDeviceSynchronize();
    return 0;
}
EOF

if nvcc $TEST_DIR/test.cu -o $TEST_DIR/test &> /dev/null; then
    echo -e "${GREEN}CUDA compilation test: PASSED${NC}"
    if $TEST_DIR/test &> /dev/null; then
        echo -e "${GREEN}CUDA runtime test: PASSED${NC}"
    else
        echo -e "${RED}CUDA runtime test: FAILED${NC}"
    fi
else
    echo -e "${RED}CUDA compilation test: FAILED${NC}"
fi

rm -rf $TEST_DIR

echo ""
echo "============================================"
echo "Recommendations for FAR Planner:"
echo "============================================"

# Get compute capability and suggest CMake setting
if [ -n "$COMPUTE_CAP" ]; then
    COMPUTE_ARCH=$(echo "$COMPUTE_CAP" | tr -d '.')
    echo "Add this to boundary_handler/CMakeLists.txt:"
    echo "  set(CMAKE_CUDA_ARCHITECTURES $COMPUTE_ARCH)"
fi

echo ""
echo "To build with GPU support:"
echo "  cd ~/Documents/Far_planner_test/workspaces/far_planner/"
echo "  colcon build --packages-select boundary_handler --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "  source install/setup.bash"

echo ""
echo "To enable GPU in config:"
echo "  Edit: boundary_handler/config/default.yaml"
echo "  Set: use_gpu: true"

echo ""
echo "============================================"
