^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_sm_loop_closure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-03-03)
------------------
* Fix build against different gtsam versions
* Add optional generation of .3Dscene files with loop-closure visualizations
* Tolerate missing external files without throwing
* Better logging and expose more env var params
* Add GNSS uncertainty multiplier
* Use debug files prefix
* Add more frame-to-frame LC selection algorithms
* Merge pull request `#4 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/4>`_ from MOLAorg/feat/f2f-fixes
  Add more parameters to f2f method
* Add new params for f2f method
* pipeline file: expose more params and fix reversed sigma logic
* Fix: prior should be weak for GNSS to transform to ENU frame
* fix copyright headers
* Merge pull request `#3 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/3>`_ from MOLAorg/feat/new-f2f-algo
  Add alternative frame-to-frame LC algorithm
* Provide virtual LoopClosureInterface to select the algorithm from a CLI argument
* Add alternative frame-to-frame LC algorithm
* Merge pull request `#2 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/2>`_ from MOLAorg/feat/add-ci-and-badges
  Add CI workflows and clang-format linting infrastructure
* package.xml: add FILE tag
* Merge pull request `#1 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/1>`_ from MOLAorg/feature/refactor-pkg-mola-gtsam-factors
  Progress moving code to new package mola_gtsam_factors
* Update dependencies after refactorization into mola_georeferencing
* Progress moving code to new package mola_gtsam_factors
* Protect against invalid GPS cov entries
* Add param gnss_minimum_uncertainty_xyz; dump trajectories as TUM files.
* simplify: remove obsolete parameters not used anymore here
* Remove unused parameter (GPS observations are automatically detected anyway)
* Fix: ensure minimum uncertainty in edges
* Fix deskewing generating sm2mm for local maps
* Implement missing loading of new LocalVelocityBuffer while doing submaps "sm2mm"
* Don't use absolute paths for debug icplog files
* FIX: Detect the error case of no GNSS observations in sm-georeference-cli
* Add gicp pipeline
* bump minimum cmake version to 3.5
* add .clang-tidy rules and better vscode integration
* Expose more env vars in the pipeline
* robust kernels; optional horizontality
* progress with submaps
* implement submaps
* use new mrpt enu covariance field
* done georeferencing cli
* progress
* progress cli app interface
* initial structure
* Initial commit
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco
