/*
 * CUDA Type Definitions (minimal header for .cu files)
 * Copyright (C) 2025 - GPU Acceleration Implementation
 */

#ifndef GRAPH_CUDA_TYPES_H
#define GRAPH_CUDA_TYPES_H

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

// GPU-compatible data structures (plain C structs for CUDA)
struct Point3D_GPU {
    float x, y, z;
    
    CUDA_CALLABLE Point3D_GPU() : x(0), y(0), z(0) {}
    CUDA_CALLABLE Point3D_GPU(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    
    CUDA_CALLABLE inline float norm_flat() const {
        return sqrtf(x * x + y * y);
    }
    
    CUDA_CALLABLE inline Point3D_GPU normalize_flat() const {
        float norm = norm_flat();
        if (norm > 1e-7f) {
            return Point3D_GPU(x / norm, y / norm, 0.0f);
        }
        return Point3D_GPU(0, 0, 0);
    }
    
    CUDA_CALLABLE inline Point3D_GPU operator-(const Point3D_GPU& other) const {
        return Point3D_GPU(x - other.x, y - other.y, z - other.z);
    }
    
    CUDA_CALLABLE inline Point3D_GPU operator+(const Point3D_GPU& other) const {
        return Point3D_GPU(x + other.x, y + other.y, z + other.z);
    }
    
    CUDA_CALLABLE inline Point3D_GPU operator*(float scalar) const {
        return Point3D_GPU(x * scalar, y * scalar, z * scalar);
    }
    
    CUDA_CALLABLE inline float operator*(const Point3D_GPU& other) const {
        return x * other.x + y * other.y;  // Flat dot product
    }
};

struct NavNode_GPU {
    Point3D_GPU position;
    int id;
    int free_direct;
    Point3D_GPU surf_dir_first;
    Point3D_GPU surf_dir_second;
    bool is_active;
    bool is_boundary;
    // Contour connection info (fixed size for GPU)
    int num_contour_connects;
    int contour_connects[100];  // Max 100 contour connections
};

struct PolygonVertex_GPU {
    Point3D_GPU position;
    int polygon_id;
    int vertex_index;
};

#endif // GRAPH_CUDA_TYPES_H
