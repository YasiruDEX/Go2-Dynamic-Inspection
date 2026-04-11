[![CI Check clang-format](https://github.com/MOLAorg/mola_imu_preintegration/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_imu_preintegration/actions/workflows/check-clang-format.yml)
[![CI ROS](https://github.com/MOLAorg/mola_imu_preintegration/actions/workflows/build-ros.yml/badge.svg)](https://github.com/MOLAorg/mola_imu_preintegration/actions/workflows/build-ros.yml)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)]([https://docs.mola-slam.org/latest/](https://docs.mola-slam.org/latest/mola_state_estimators.html))


| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_imu_preintegration__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_imu_preintegration__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_imu_preintegration__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_imu_preintegration__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_imu_preintegration__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mola_imu_preintegration)](https://index.ros.org/?search_packages=true&pkgs=mola_imu_preintegration) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mola_imu_preintegration__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_imu_preintegration__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_imu_preintegration__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_imu_preintegration__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_imu_preintegration__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_imu_preintegration__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_imu_preintegration)](https://index.ros.org/?search_packages=true&pkgs=mola_imu_preintegration) |
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mola_imu_preintegration__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_imu_preintegration__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mola_imu_preintegration__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mola_imu_preintegration__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_imu_preintegration__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_imu_preintegration__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mola_imu_preintegration)](https://index.ros.org/?search_packages=true&pkgs=mola_imu_preintegration) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_imu_preintegration__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_imu_preintegration__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__mola_imu_preintegration__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mola_imu_preintegration__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_imu_preintegration__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_imu_preintegration__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_imu_preintegration)](https://index.ros.org/?search_packages=true&pkgs=mola_imu_preintegration) |

# mola_imu_preintegration
A lightweight package for IMU preintegration routines, IMU data manipulation, and basic IMU calibration.

This repository provides:
* `mola::imu::IMUIntegrator`: A C++ class to integrate IMU accelerations and angular velocities.
* `mola::imu::ImuTransformer`: Transforms IMU readings from an arbitrary sensor pose to the vehicle reference `base_link`.
* `mola::imu::LocalVelocityBuffer`: Holds a short window of local velocities, accelerations, and global orientation, from external estimators and an IMU.
* `mola::imu::trajectory_from_buffer()`: Reconstruct a trajectory from a `LocalVelocityBuffer`.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the GNU GPL v3 license. Other options available upon request.

```cpp
/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola
*/
```
