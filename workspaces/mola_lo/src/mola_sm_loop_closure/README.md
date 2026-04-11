[![CI ROS](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/ros-build.yml/badge.svg)](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/ros-build.yml)
[![CI Check clang-format](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/check-clang-format.yml)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/mola_sm_loop_closure/)


| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_sm_loop_closure__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_sm_loop_closure__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_sm_loop_closure__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_sm_loop_closure__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_sm_loop_closure__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_sm_loop_closure__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mola_sm_loop_closure__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_sm_loop_closure__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) | 
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mola_sm_loop_closure__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_sm_loop_closure__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) | 
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_sm_loop_closure__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_sm_loop_closure__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) |



# mola_sm_loop_closure
Offline tool for loop-closure on simple-maps

## Georeferencing a map

```bash
# Create the mm:
sm2mm -i INPUT_WITH_GPS.simplemap \
 -o MAP.mm \
 -p pipeline.yaml

# georeference it:
mola-sm-georeferencing -i INPUT_WITH_GPS.simplemap --write-into MAP.mm
```

# Loop closure detection

Example usage:

```bash
mola-sm-lc-cli \
 --pipeline src/mola_sm_loop_closure/pipelines/loop-closure-lidar3d.yaml \
 -i map_KAIST01_gps.simplemap \
 -o map_KAIST01_corrected.simplemap
```

# License
Copyright (C) 2018-2026 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main 
intention of being useful for research and evaluation purposes.
Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).
