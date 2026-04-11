^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_state_estimation_smoother
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2026-03-03)
------------------
* Merge pull request `#14 <https://github.com/MOLAorg/mola_state_estimation/issues/14>`_ from MOLAorg/feature/use-imu-for-georef-hint
  Feature/use-imu-for-georef-hint
* clang-format
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2026-02-14)
------------------
* Merge pull request `#13 <https://github.com/MOLAorg/mola_state_estimation/issues/13>`_ from MOLAorg/feat/publish-georef-on-converge
  Publish geo-ref on convergence
* get onObservation() and has_converged_localization() method tested too
* Refactor publish georef and sensor label regex
  fix filtering observations by sensor label regex
  refactor to publish georef on converged automatically
  fix build against older mola_kernel
  Fix wrong logic in GNSS fuse
* remove old mola version guards
* unit test: relax threshold to avoid spurious failures
* Publish geo-ref on convergence
  add new params to yaml file
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2026-01-15)
------------------
* test: less strict limits to avoid random failures
* cmake: Remove non-used find_package() for ament_lint_auto
* Merge pull request `#11 <https://github.com/MOLAorg/mola_state_estimation/issues/11>`_ from MOLAorg/feat/improve-docs
  Improve docs for mola_gtsam_factors
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2026-01-14)
------------------
* Merge pull request `#7 <https://github.com/MOLAorg/mola_state_estimation/issues/7>`_ from MOLAorg/feature/tricycle-kinematic
  Add tricycle kinematics
* Use bad initial pose for unit test
* unit test: same conditions for different kinematic models
* Copyright year bump
* Add FactorTricycleKinematic
* Implement twist fusion
* Add twist unit test
* Unit test: include tricycle tests too
* Enable connection to MolaViz for console messages
* converted into ament_cmake; update template yaml params file
* Add templates to launch the estimator as a standalone ROS 2 node
* Merge pull request `#5 <https://github.com/MOLAorg/mola_state_estimation/issues/5>`_ from MOLAorg/feature/fuse-gnss-imu-odom
  Refactor: new packages mola_georeferencing, mola_gtsam_factors, functional smoother state estimator
* More realistic drift case: IMU helps LO
* fix test: correct IMU acc simulation
* Default params: more common case of starting near static
* fixed imu acc gravity alignment
* Add two-odometry unit test
* fix error, and add support for azimuth offset
* add IMU+GPS  based azimuth estimation
* Refactor: symbolic factor classes moved as internal smoother classes
* Make mola_gtsam_factors non headers-only and rename GNSS2ENU as GnssEnu for consistency
* fixed missing extrapolation steps
* implement basic pose extrapolation
* honor initial twist
* Fixed odom+gnss fusion
* implement correct auto geo-referenciation
* Support fusing poses wrt map frame too
* marginals for uncertainty
* refactor to use Pose3 instead of P+R
* initialize georef gtsam variables
* Make important parameters mandatory in yaml config files
* New unit test for fusing GPS+odometry
* process CObservationRobotPose
* One further fix for cmake
* integrate code coverage in cmake
* Update .h docs to match the new design
* Enable code coverage
* Update parameters to hold new geo-ref fields
* Refactor to expose all gtsam factors into a new library 'mola_gtsam_factors'
* Contributors: Jose Luis Blanco-Claraco

1.11.1 (2025-10-20)
-------------------
* Update to build against MOLA>=2.1.0 with ConstPtr API
* Contributors: Jose Luis Blanco-Claraco

1.11.0 (2025-10-05)
-------------------
* Move LocalVelocityBuffer class here from mp2p_icp repository
* Contributors: Jose Luis Blanco-Claraco

1.10.0 (2025-09-07)
-------------------
* Fix build against gtsam>=4.3
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
* Update copyright year
* fixes for clang-tidy
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-03-15)
------------------
* const correctness
* State estimation modules now are proper MOLA raw inputs, so they automatically subscribe and consume input sensors (IMU, GPS, wheels odometry)
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-02-22)
------------------
* Use more generic localization source name
* make it thread safe; fix replaying extrapolated poses using past timestamps
* Documentation: explain the different types of factors and kinematic models
* Smoother: observe the enforce_planar_motion parameter
* FIX: use last guess as initial values to improve optimization stability; expose more parameters
* StateEstimationSmoother: Publish pose updates in a timely manner
* Add parameter enforce_planar_motion
* Fix gtsam must be a runtime depend too
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

1.6.1 (2025-01-10)
------------------
* Shorter logger name
* Contributors: Jose Luis Blanco-Claraco

1.6.0 (2025-01-03)
------------------

1.5.0 (2024-12-26)
------------------

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------

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
* Update test-navstate-basic.cpp: less noisy test data for more predictable results
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
* Fix GNSS typo
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------
* Create new NavStateFilter interface and separate the simple fuser and the factor-graph approach in two packages
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
