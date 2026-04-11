^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_gtsam_factors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2026-03-03)
------------------

2.1.0 (2026-02-14)
------------------

2.0.1 (2026-01-15)
------------------
* Fix potential bug in sm_georeferencing
* Merge pull request `#11 <https://github.com/MOLAorg/mola_state_estimation/issues/11>`_ from MOLAorg/feat/improve-docs
  Improve docs for mola_gtsam_factors
* Improve docs for mola_gtsam_factors
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2026-01-14)
------------------
* Merge pull request `#7 <https://github.com/MOLAorg/mola_state_estimation/issues/7>`_ from MOLAorg/feature/tricycle-kinematic
  Add tricycle kinematics
* fix jacobians
* remove now unused ctor param
* Fix build for gtsam>=4.3
* Copyright year bump
* Add FactorTricycleKinematic
* Merge pull request `#9 <https://github.com/MOLAorg/mola_state_estimation/issues/9>`_ from MOLAorg/fix/build-deps
  Fix wrong dep name
* Fix wrong dep name
* Merge pull request `#5 <https://github.com/MOLAorg/mola_state_estimation/issues/5>`_ from MOLAorg/feature/fuse-gnss-imu-odom
  Refactor: new packages mola_georeferencing, mola_gtsam_factors, functional smoother state estimator
* fixed imu acc gravity alignment
* add IMU+GPS  based azimuth estimation
* Refactor: symbolic factor classes moved as internal smoother classes
* Make mola_gtsam_factors non headers-only and rename GNSS2ENU as GnssEnu for consistency
* implement correct auto geo-referenciation
* refactor to use Pose3 instead of P+R
* One further fix for cmake
* integrate code coverage in cmake
* package.xml: add FILE to license tags
* Fix build with gtsam>2.3
* Refactor to expose all gtsam factors into a new library 'mola_gtsam_factors'
* Contributors: Jose Luis Blanco-Claraco
