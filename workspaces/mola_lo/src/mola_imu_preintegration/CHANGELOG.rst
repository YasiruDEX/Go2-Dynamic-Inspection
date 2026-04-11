^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_imu_preintegration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.0 (2026-02-23)
-------------------
* Merge pull request `#1 <https://github.com/MOLAorg/mola_imu_preintegration/issues/1>`_ from MOLAorg/feat/transform-without-w
  ImuTransformer: work on acceleration even without omega
* BUG FIX: Wrong copy-paste sign error in centripetal acceleration
* ImuTransformer: work on acceleration even without omega
* Contributors: Jose Luis Blanco-Claraco

1.14.1 (2025-12-26)
-------------------
* Add option 'use_imu_orientation' to disable using IMU orientation for initialization
* CI: Use standard checkout action
* Contributors: Jose Luis Blanco-Claraco

1.14.0 (2025-11-11)
-------------------
* BUGFIX: Estimated acc. bias and initial roll had a bug in their formulas.
* Add unit tests for initial calibrator
* Update project website URL in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.13.2 (2025-10-20)
-------------------
* Make use of ConstPtr in API
* Contributors: Jose Luis Blanco-Claraco

1.13.1 (2025-10-15)
-------------------
* ImuCalibrator: Robust against invalid orientation data in Imu msgs
* Fix random failures in unit test
* CI: add running unit tests
* Contributors: Jose Luis Blanco-Claraco

1.13.0 (2025-10-11)
-------------------
* FIX: Picked wrong reference stamp for integrating trajectories
* Add asString() debugging methods to sample structs
* Contributors: Jose Luis Blanco-Claraco

1.12.0 (2025-10-09)
-------------------
* FIX: May leave trajectory poses without populating raw IMU data
* ImuInitialCalibrator: Use IMU orientation, if available
* LocalVelocityBuffer: reduce default max_time_window to 0.5 s
* ImuIntegrationParams: Add save_to() method, and finish missing load_from() fields
* Fix comment typos
* Readme: Add missing ROS build farm badges for bin packages
* Contributors: Jose Luis Blanco-Claraco

1.11.0 (2025-09-24)
-------------------
* New unit tests for IMU integration
* Move everything into namespace mola::imu to avoid ns pollution
* Move LocalVelocityBuffer class here from mp2p_icp repository
* Contributors: Jose Luis Blanco-Claraco

1.10.0 (2025-09-07)
-------------------
* Add class ImuTransformer
* Update copyright notice
* Import IMU initialization class ImuInitialCalibrator, refactored from the mola LO package
* Contributors: Jose Luis Blanco-Claraco

1.9.0 (2025-06-06)
------------------

1.8.1 (2025-05-25)
------------------
* fixes for clang-tidy
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-03-15)
------------------

1.7.0 (2025-02-22)
------------------

1.6.1 (2025-01-10)
------------------
* Fix package.xml URLs
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
* Update clang-format style; add reformat bash script
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

1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Update docs
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* API changes for new package mola_navstate_fuse
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Update copyright year
* Correct references to license
* Ported to ROS2 colcon build system
* Delete WIP files.
* first unit tests
* progress, unit tests
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2021-09-18)
------------------
* Initial commit
* Contributors: Jose Luis Blanco-Claraco
