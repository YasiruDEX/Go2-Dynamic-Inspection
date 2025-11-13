/*
 * CUDA Type Definitions for FAR Planner GPU Acceleration
 * Copyright (C) 2025 - GPU Acceleration Implementation
 */

#ifndef CUDA_TYPES_H
#define CUDA_TYPES_H

#include <cmath>

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

// GPU-compatible data structures
struct Point3D_GPU {
    float x, y, z;
    
    CUDA_CALLABLE Point3D_GPU() : x(0), y(0), z(0) {}
    CUDA_CALLABLE Point3D_GPU(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    
    CUDA_CALLABLE inline float norm() const {
        return sqrtf(x * x + y * y + z * z);
    }
    
    CUDA_CALLABLE inline float norm_flat() const {
        return sqrtf(x * x + y * y);
    }
    
    CUDA_CALLABLE inline Point3D_GPU normalize() const {
        float n = norm();
        if (n > 1e-7f) {
            return Point3D_GPU(x / n, y / n, z / n);
        }
        return Point3D_GPU(0, 0, 0);
    }
    
    CUDA_CALLABLE inline Point3D_GPU normalize_flat() const {
        float n = norm_flat();
        if (n > 1e-7f) {
            return Point3D_GPU(x / n, y / n, 0.0f);
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
    int free_direct;  // 0=UNKNOWN, 1=CONVEX, 2=CONCAVE, 3=PILLAR
    Point3D_GPU surf_dir_first;
    Point3D_GPU surf_dir_second;
    bool is_boundary;
    bool is_odom;
    bool is_navpoint;
    // Contour connection info (fixed size for GPU)
    int num_contour_connects;
    int contour_connects[50];  // Max 50 contour connections
};

struct PolygonEdge_GPU {
    Point3D_GPU start;
    Point3D_GPU end;
    int polygon_id;
    float height_min;
    float height_max;
    bool is_boundary;
};

#endif // CUDA_TYPES_H
