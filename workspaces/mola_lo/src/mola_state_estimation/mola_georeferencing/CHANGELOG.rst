^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_georeferencing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2026-03-03)
------------------
* Merge pull request `#18 <https://github.com/MOLAorg/mola_state_estimation/issues/18>`_ from MOLAorg/fix/should-fail-on-missing-mm
  Add guards against failures for missing input .mm files
* Add guards against failures for missing input .mm files
* Merge pull request `#17 <https://github.com/MOLAorg/mola_state_estimation/issues/17>`_ from MOLAorg/feat/mm-add-geodetic-handle-null-geodetics
  mola-mm-add-geodetic: Don't create output map if input geodetics are …
* mola-mm-add-geodetic: Don't create output map if input geodetics are missing.
  We may still have a valid input .georef, obtained from IMU only, but without geodetics coordinates
* Merge pull request `#16 <https://github.com/MOLAorg/mola_state_estimation/issues/16>`_ from MOLAorg/feat/imu-align-without-gps
  Use IMU even without GPS data
* improved unit tests
* fix potential UB
* Use IMU even without GPS data
* IMU frames: use averaged gravity measurements per key-frame
* Merge pull request `#15 <https://github.com/MOLAorg/mola_state_estimation/issues/15>`_ from MOLAorg/fix/use-lvb
  georeferencing: correctly use IMU data in the LocalVelocityBuffer too…
* georeferencing: correctly use IMU data in the LocalVelocityBuffer too for gravity alignment
* Merge pull request `#14 <https://github.com/MOLAorg/mola_state_estimation/issues/14>`_ from MOLAorg/feature/use-imu-for-georef-hint
  Feature/use-imu-for-georef-hint
* Fix potential errors
* georeferencing now optionally uses IMU acceleration measurements to help with gravity-alignment
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2026-02-14)
------------------
* simplemap_georeference(): returns std::optional to reflect lack of GNSS data enough
* Merge pull request `#12 <https://github.com/MOLAorg/mola_state_estimation/issues/12>`_ from MOLAorg/feat/mm-geodetic
  Feat/mm-geodetic
* Add mola-mm-add-geodetic-cli app
* Contributors: Jose Luis Blanco-Claraco

2.0.1 (2026-01-15)
------------------
* Fix potential bug in sm_georeferencing
* Merge pull request `#11 <https://github.com/MOLAorg/mola_state_estimation/issues/11>`_ from MOLAorg/feat/improve-docs
  Improve docs for mola_gtsam_factors
* Add missing #include for gtsam
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2026-01-14)
------------------
* Merge pull request `#7 <https://github.com/MOLAorg/mola_state_estimation/issues/7>`_ from MOLAorg/feature/tricycle-kinematic
  Add tricycle kinematics
* lint clean ups
* Copyright year bump
* Merge pull request `#10 <https://github.com/MOLAorg/mola_state_estimation/issues/10>`_ from MOLAorg/feature/add-georef-tests
  Add georeference unit tests
* Add georef unit test
* Merge pull request `#5 <https://github.com/MOLAorg/mola_state_estimation/issues/5>`_ from MOLAorg/feature/fuse-gnss-imu-odom
  Refactor: new packages mola_georeferencing, mola_gtsam_factors, functional smoother state estimator
* Make mola_gtsam_factors non headers-only and rename GNSS2ENU as GnssEnu for consistency
* process CObservationRobotPose
* integrate code coverage in cmake
* package.xml: add FILE to license tags
* Refactor to expose all gtsam factors into a new library 'mola_gtsam_factors'
* new package mola_georeferencing, imported and refactored from mola_sm_loop_closure
* Contributors: Jose Luis Blanco-Claraco
