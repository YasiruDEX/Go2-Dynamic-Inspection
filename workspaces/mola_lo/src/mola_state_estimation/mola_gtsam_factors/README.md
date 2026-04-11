# mola_gtsam_factors

C++ library providing reusable GTSAM (Georgia Tech Smoothing and Mapping) factors for georeferencing and state estimation in robotics and autonomous systems.

## Overview

This library is part of the [MOLA (Modular Optimization framework for Localization and mApping)](https://docs.mola-slam.org/) project and provides custom factor graph nodes that can be used with GTSAM for:

- **Sensor fusion**: Combine IMU, GNSS, wheel odometry, and other sensors
- **State estimation**: Track robot pose, velocities, and orientation over time
- **Georeferencing**: Align local maps with global coordinate systems
- **Motion modeling**: Apply kinematic and dynamic constraints

## Features

### Motion Integration Factors
- **FactorAngularVelocityIntegration**: Integrates angular velocity measurements to constrain orientation changes
- **FactorTrapezoidalIntegrator**: Trapezoidal rule integration for velocity-to-position constraints
- **FactorTricycleKinematic**: Kinematic model for tricycle-like vehicles (cars, Ackermann steering)

### Velocity Constraints
- **FactorConstLocalVelocity**: Enforces constant velocity assumption in the body frame (useful for smoothing)

### GNSS/GPS Factors
- **FactorGnssEnu**: GNSS observations in East-North-Up (ENU) coordinates
- **FactorGnssMapEnu**: GNSS with explicit ENU-to-map transform optimization

### IMU/Orientation Factors
- **MeasuredGravityFactor**: Gravity-aligned orientation from accelerometer measurements
- **Pose3RotationFactor**: Constraints on rotation only (decoupled from translation)

## Installation

### ROS 2 (Recommended)

```bash
# Using apt (if available in your ROS distro)
sudo apt install ros-${ROS_DISTRO}-mola-gtsam-factors

# Or build from source in your workspace
cd ~/ros2_ws/src
git clone https://github.com/MOLAorg/mola_state_estimation.git
cd ~/ros2_ws
colcon build --packages-select mola_gtsam_factors
```

### Standalone CMake

```bash
# Install dependencies
sudo apt install libgtsam-dev libmrpt-poses-dev

# Clone and build
git clone https://github.com/MOLAorg/mola_state_estimation.git
cd mola_state_estimation/mola_gtsam_factors
mkdir build && cd build
cmake ..
make
sudo make install
```

## Dependencies

- **GTSAM** (>= 4.0): Factor graph optimization library
- **MRPT** (mrpt-poses): Mobile Robot Programming Toolkit for pose representations
- **mola_common**: Common MOLA utilities and CMake scripts

## Quick Start

### Example: Using Angular Velocity Integration

```cpp
#include <mola_gtsam_factors/FactorAngularVelocityIntegration.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

using namespace mola::factors;

// Create factor graph
gtsam::NonlinearFactorGraph graph;

// Define keys for rotations and angular velocity
gtsam::Key kR0 = gtsam::Symbol('R', 0);
gtsam::Key kW0 = gtsam::Symbol('W', 0);
gtsam::Key kR1 = gtsam::Symbol('R', 1);

// Create noise model (3D rotation uncertainty)
auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

// Add factor: R1 should be R0 rotated by angular velocity W0 over time dt
double dt = 0.1; // 100ms
graph.add(FactorAngularVelocityIntegration(kR0, kW0, kR1, dt, noise));

// Initialize values and optimize...
```

### Example: GNSS Positioning

```cpp
#include <mola_gtsam_factors/FactorGnssEnu.h>

// GNSS observation in ENU coordinates
gtsam::Point3 gnss_observation(100.5, 200.3, 10.2); // East, North, Up

// Sensor offset from vehicle center
gtsam::Point3 antenna_offset(0.5, 0.0, 1.2); // meters

// Create GNSS factor
auto gnss_noise = gtsam::noiseModel::Isotropic::Sigma(3, 2.0); // 2m std dev
graph.add(FactorGnssEnu(
    kPose,              // Vehicle pose key
    antenna_offset,     // Antenna location on vehicle
    gnss_observation,   // Measured ENU position
    gnss_noise
));
```

### Example: Tricycle Kinematic Model

```cpp
#include <mola_gtsam_factors/FactorTricycleKinematic.h>

// Model vehicle motion as a circular arc
gtsam::Key kT0 = gtsam::Symbol('T', 0);  // Initial pose
gtsam::Key kV0 = gtsam::Symbol('V', 0);  // Linear velocity
gtsam::Key kW0 = gtsam::Symbol('W', 0);  // Angular velocity
gtsam::Key kT1 = gtsam::Symbol('T', 1);  // Final pose

auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
double dt = 0.1; // time step

graph.add(FactorTricycleKinematic(kT0, kV0, kW0, kT1, dt, noise));
```

## Factor Reference

### Motion Models

| Factor | Description | Variables | Use Case |
|--------|-------------|-----------|----------|
| `FactorAngularVelocityIntegration` | ω integration: R₁ = R₀ ⊕ Exp(ω·dt) | 2 Rot3 (rotations), 1 Vector3 (angular vel) | IMU gyroscope integration |
| `FactorTrapezoidalIntegrator` | Trapezoidal velocity integration | 2 poses, 2 velocities | Smooth velocity-based motion |
| `FactorTricycleKinematic` | Ackermann/car kinematic model | 1 pose, 2 velocities, 1 pose | Wheeled vehicle motion |
| `FactorConstLocalVelocity` | Constant body-frame velocity | 2 rotations, 2 velocities | Velocity smoothing prior |

### Positioning & Orientation

| Factor | Description | Variables | Use Case |
|--------|-------------|-----------|----------|
| `FactorGnssEnu` | GNSS position in ENU | 1 Pose3 | GPS/GNSS measurements |
| `FactorGnssMapEnu` | GNSS with ENU↔map transform | 2 Pose3 (transform, pose) | Map georeferencing |
| `MeasuredGravityFactor` | Gravity direction constraint | 2 Pose3 (transform, pose) | IMU accelerometer |
| `Pose3RotationFactor` | Rotation-only measurement | 2 Pose3 (transform, pose) | Compass, orientation sensors |

## Documentation

- **API Documentation**: [docs.mola-slam.org](https://docs.mola-slam.org/latest/mola_state_estimators.html)
- **Tutorials**: See the `mola_state_estimation_smoother` package for usage examples
- **Factor Graphs**: [GTSAM Tutorial](https://gtsam.org/tutorials/intro.html)

## Contributing

Contributions are welcome! Please:

1. Follow the existing code style (enforced by clang-format)
2. Add tests for new factors
3. Update documentation for API changes
4. Submit pull requests to the main repository

## Citation

If you use this library in academic work, please cite:

```bibtex
@misc{mola_gtsam_factors,
  author = {Blanco-Claraco, Jose Luis},
  title = {MOLA GTSAM Factors: Reusable Factor Graph Components},
  year = {2025},
  publisher = {GitHub},
  url = {https://github.com/MOLAorg/mola_state_estimation}
}
```

## License

Copyright (C) 2018-2026 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the **GNU GPL v3** license as open source, primarily intended for research and evaluation purposes.
Commercial licenses are [available upon request](https://docs.mola-slam.org/latest/solutions.html) for proprietary applications.

## Support

- **Documentation**: https://docs.mola-slam.org/
- **Issues**: https://github.com/MOLAorg/mola_state_estimation/issues
