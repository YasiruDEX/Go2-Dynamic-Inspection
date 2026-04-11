^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_state_estimation_simple
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2026-03-03)
------------------

2.1.0 (2026-02-14)
------------------
* Merge pull request `#13 <https://github.com/MOLAorg/mola_state_estimation/issues/13>`_ from MOLAorg/feat/publish-georef-on-converge
  Publish geo-ref on convergence
* Include the RDC API in the tests
* Refactor publish georef and sensor label regex
  fix filtering observations by sensor label regex
  refactor to publish georef on converged automatically
  fix build against older mola_kernel
  Fix wrong logic in GNSS fuse
* remove old mola version guards
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2026-01-15)
------------------

2.0.0 (2026-01-14)
------------------
* Merge pull request `#7 <https://github.com/MOLAorg/mola_state_estimation/issues/7>`_ from MOLAorg/feature/tricycle-kinematic
  Add tricycle kinematics
* lint clean ups
* Copyright year bump
* Merge pull request `#8 <https://github.com/MOLAorg/mola_state_estimation/issues/8>`_ from MOLAorg/fix/add-simple-estimator-unit-tests
  Add tests for simple estimator
* Add tests for simple estimator
* Merge pull request `#5 <https://github.com/MOLAorg/mola_state_estimation/issues/5>`_ from MOLAorg/feature/fuse-gnss-imu-odom
  Refactor: new packages mola_georeferencing, mola_gtsam_factors, functional smoother state estimator
* One further fix for cmake
* integrate code coverage in cmake
* Refactor to expose all gtsam factors into a new library 'mola_gtsam_factors'
* Contributors: Jose Luis Blanco-Claraco

1.11.1 (2025-10-20)
-------------------
* Update to build against MOLA>=2.1.0 with ConstPtr API
* Contributors: Jose Luis Blanco-Claraco

1.11.0 (2025-10-05)
-------------------
* Move mola_imu_preintegration out of this repo
  The new repository is: https://github.com/MOLAorg/mola_imu_preintegration
* Merge pull request `#4 <https://github.com/MOLAorg/mola_state_estimation/issues/4>`_ from MOLAorg/feature/imu
  Refactor IMU library
* Fix using new IMU integration API
* Move LocalVelocityBuffer class here from mp2p_icp repository
* Contributors: Jose Luis Blanco-Claraco

1.10.0 (2025-09-07)
-------------------
* Update copyright notice
* Make unhandled sensor input topic message less verbose
* Contributors: Jose Luis Blanco-Claraco

1.9.0 (2025-06-06)
------------------
* State estimation interface is now raw data consumer too
* FIX: Error if sensor labels were provided in config yaml file
* Contributors: Jose Luis Blanco-Claraco

1.8.1 (2025-05-25)
------------------
* Feature: Implement basic twist covariance handling in the SimpleEstimator
* fixes for clang-tidy
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-03-15)
------------------
* const correctness
* State estimation modules now are proper MOLA raw inputs, so they automatically subscribe and consume input sensors (IMU, GPS, wheels odometry)
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-02-22)
------------------
* Add parameter enforce_planar_motion
* Contributors: Jose Luis Blanco-Claraco

1.6.1 (2025-01-10)
------------------
* Merge pull request `#1 <https://github.com/MOLAorg/mola_state_estimation/issues/1>`_ from MOLAorg/9-need-help-on-integrating-imu-for-lidar-odometry
  More stable integration of IMU twist information
* Shorter logger name
* Fix package.xml URLs
* tolerate unsorted sensor inputs without throwing
* Contributors: Jose Luis Blanco-Claraco

1.6.0 (2025-01-03)
------------------
* Simple estimator: Integrate IMU angular velocity readings
* Contributors: Jose Luis Blanco-Claraco

1.5.0 (2024-12-26)
------------------

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------
* Allow zero variances for some pose components
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2024-12-11)
------------------
* Start integrating GNSS observation. Added a new CLI program mola-navstate-cli for testing state fusion
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------

1.1.3 (2024-08-28)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2024-08-26)
------------------

1.1.1 (2024-08-23)
------------------

1.1.0 (2024-08-18)
------------------
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------

1.0.6 (2024-06-21)
------------------
* Create new NavStateFilter interface and separate the simple fuser and the factor-graph approach in two packages
* Twist in local frame
* Contributors: Jose Luis Blanco-Claraco

1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* use odometry
* add new package mola_state_estimation_simple
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
