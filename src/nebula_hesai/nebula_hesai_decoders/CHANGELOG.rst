^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nebula_hesai_decoders
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-04-06)
------------------

0.4.0 (2026-03-27)
------------------
* chore: remove PCL dependency (`#401 <https://github.com/tier4/nebula/issues/401>`_)
  * chore: remove all PCL deps from package.xmls
  * chore: remove PCL from all CMakeLists.txts
  * chore: remove unused PointXYZICATR
  * chore: remove pcl from nebula_core
  * chore: transfer continental point types to new format
  * add xyz point type
  * chore: add xyzircaedt to xyz conversion
  * chore: add pcd I/O
  * chore: add Nebula PointCloud to PointCloud2 conversion
  * refactor: remove PCL usage completely
  * refactor: clean PCD I/O
  * test: pcd I/O unit tests
  * chore: update copyright year to 2026
  * chore(pcd): handle padding and alignment correctly
  * perf: make all point types aligned to 16B boundaries
  * fix(ars548): fix jazzy build failure
  * fix(continental): more missing header fixes for Jazzy
  * build: add missing Boost dependency across multiple components
  * fix: correct headers and call sites for convert_point\_... functions
  ---------
* Contributors: Max Schmeller

0.3.2 (2026-01-27)
------------------

0.3.1 (2026-01-19)
------------------
* fix(hesai): fix scan cutting for `cut_angle != cloud_max_angle` configurations (`#383 <https://github.com/tier4/nebula/issues/383>`_)
  * chore: estable encoder/spatial terminology
  * chore: better type inference for normalize_angle
  * fix: change angle handling to integer math to eliminate rounding errors
  * fix: properly fixed scan cutting
  * feat: properly isolated scan cutter, works now
  * fix: add blockage mask publishing back in
  * test: scan cutter unit tests
  * fix: fixed most failing tests
  * chore: add validation in scan_cutter constructor
  * docs: support drawio, mathjax
  * docs: basic scan cutting state machine logic
  * docs: wording
  * fix: implement state machine based cutter
  * test: fix invalidly configured pandar64 unit test
  * fix: reset decoder when rosbag loops
  * fix: fix wrong correction term for at128
  * fix: re-introduce decode perf counters
  * perf: faster normalize_angle for integral types
  * perf: prevent runtime type conversion in deg2rad/rad2deg
  * chore: make NebulaPoint aligned like the rest of the point types
  * chore: random small fixes
  * perf: faster angle normalization
  * perf: avoid re-computing fov states
  * chore: make angle_is_between inline for good measure
  * chore: update profiling scripts to work with latest nebula
  * test: add angle unit tests
  * fix: fix normalize_angle for negative ints
  * build: add angles test to cmake
  * chore: hide uv.lock changes in diff
  * chore: remove unused scan cutter param
  * chore: reduce number of states in CutAtFovEnd FSM to 6
  * chore: simplify scan cutter tests
  * chore: update docs for simplified fsm
  * fix: correct channel number for minmax array
  * fix: fix find_field for AT128
  * fix: fix min/max channel-based checks to work when the max-corrected channel has lower index than the min-corrected one
  * test: allow for float rounding errors around fov bounds
  * chore: remove unused code
  * perf: re-introduce early returns in decoder
  * perf: reintroduce darian luts
  * test: update decoder ground truths to reflect new scan cutting behavior
  * chore(cspell): ignore drawio source files
  * docs: add scan cutting design diagrams, update mkdocs nav
  * chore: remove residual debug file code
  * chore: correct comment for 100ms after-cut timestamp check
  * chore: remove commented-out code
  * chore: remove uv for now since it's unrelated to this pr
  * chore: remove old test folder
  * chore: allow deg2rad for integer args
  * fix: change corrections, scan cutting back to float for precision
  * chore: remove temporary ground truth output code
  * docs: zensical drawio and latex support
  * chore: import cmath
  * chore: fix normalize_angle typing edge case
  * chore: fix copyright year
  * fix: calculate fov state correctly when block does not intersect cut
  * chore: virtual destructor for AngleCorrector
  * test: add tests for scan cutter initialization
  * chore: fix cspell
  * fix(test): fix add_packet() azimuth span accumulation bug
  The diff was computed AFTER updating end_azimuth, resulting in
  diff always being 0. This made azimuth_span_accumulated never grow,
  rendering the resilience test assertions (overlaps_itself,
  get_angular_span) ineffective.
  Fix: compute diff before updating end_azimuth.
  * fix(hesai): fix did_scan_complete flag never being set
  After refactoring scan completion to use ScanCutter callbacks, the
  did_scan_complete flag was left as a local variable initialized to
  false and never updated.
  Fix:
  - Add did_scan_complete\_ member variable
  - Set it to true in on_scan_complete() callback
  - Reset it at the start of each unpack() call
  - Use it when populating PacketMetadata
  Also fix callback_time_ns\_ to accumulate (+=) instead of overwrite (=)
  in case multiple scans complete within a single packet.
  * fix(hesai): use current block for timestamp reset calculation
  Previously, on_set_timestamp() always used block 0 for the point time
  offset calculation. This was incorrect because the timestamp reset
  happens at the block where the reset angle was detected, not at block 0.
  Using block 0 could shift scan_timestamp_ns earlier than the first
  point of the new scan by up to the packet span (for sensors with
  negative per-point offsets like Pandar64).
  Fix: Track current_block_id\_ and use it in on_set_timestamp() to get
  the correct earliest point time offset for the actual reset block.
  ---------
* test: add angle unit tests (`#387 <https://github.com/tier4/nebula/issues/387>`_)
  * test: add angle unit tests
  Cherry-picked from fix/scan-cutting-rounding-error branch.
  Add unit tests for normalize_angle and angle_is_between functions.
  * chore: clarify `angle_is_between` behavior with doc comments and clearer code
  * test: update assumptions for some tests, add new edge cases
  * fix: fixed a case where overlap region becomes empty/duplicated when all correction terms are zero
  * test: regenerate XT16 ground truth after overlap region fix
  ---------
* chore(maintaners): update maintainers in all packages (`#385 <https://github.com/tier4/nebula/issues/385>`_)
  * chore(maintaners): update maintainers in all packages
  * chore(maintainers): add authors
  * chore(maintainers): add author to Continental packages
  ---------
* Contributors: David Wong, Max Schmeller

0.3.0 (2025-12-02)
------------------
* chore: replace ament_cmake_auto with autoware_cmake (`#381 <https://github.com/tier4/nebula/issues/381>`_)
  * chore: replace ament_cmake_auto with autoware_cmake in nebula_continental
  * chore: replace amen_cmake_auto with autoware_cmake in nebula_core
  * chore: replace ament_cmake_auto with autoware_cmake in nebula_hesai
  * chore: replace ament_cmake_auto with autoware_cmake in nebula_robosense
  * chore: replace ament_cmake_auto with autoware_cmake in nebula_velodyne
  * chore: format CMakeLists.txt
  * chore: fix incorrect dependency
  ---------
* chore: flatten nebula\_<vendor>_common include structures (`#380 <https://github.com/tier4/nebula/issues/380>`_)
* refactor: independent vendor packages (`#376 <https://github.com/tier4/nebula/issues/376>`_)
  * Refactor: Split nebula_common into base and vendor-specific packages
  * Refactor: Split nebula_hw_interfaces into base and vendor-specific packages
  * Refactor nebula_decoders: Create vendor-specific decoder packages (hesai, velodyne, robosense, continental)
  * Fix duplicate nested directories in decoder packages
  * Refactor nebula_ros: Create nebula_ros_base package
  * Refactor nebula_ros: Create nebula_ros_hesai package
  * Refactor nebula_ros: Create vendor-specific ROS packages (velodyne, robosense, continental)
  * Add nebula_ros_base_common library for parameter_descriptors
  * Fix link order for hesai_ros_wrapper
  * Update remaining include paths in vendor-specific ROS packages
  * Update launch files to reference vendor-specific packages
  * Update nebula_ros_base to link Hesai test correctly
  * Move Hesai functional safety test to nebula_ros_hesai to fix circular dependency
  * Fix include paths in hw_interfaces_base test
  * Fix include paths in vendor hw_interfaces packages
  * Fix decoder base include structure and test paths
  * Fix include paths in decoder base headers
  * Add rclcpp dependency to decoder packages
  * Fix ROS base target export and include paths
  * Add smoke tests to nebula_ros_base
  * Fix nebula_examples to use new refactored packages
  * Fix config file paths and launch file references for all vendor packages - all smoke tests passing
  * .
  * ..
  * Restore Hesai functional safety test resources from deleted commit
  - Restored test_resources/fusa_codes/ CSV files that were deleted in commit aa7eb7b6
  - Moved test resources to nebula_ros_hesai package (they were previously in nebula_ros)
  - Updated CMakeLists.txt to install test_resources directory
  - All 20 functional safety tests are now passing
  * Reorganize schema files: move common files to nebula_ros_base/schema and vendor-specific files to vendor packages
  - Moved common schema files from nebula_ros_base/schema/sub/ to nebula_ros_base/schema/
  - Moved vendor-specific schema files to their respective vendor packages:
  - lidar_hesai.json -> nebula_ros_hesai/schema/
  - lidar_velodyne.json -> nebula_ros_velodyne/schema/
  - lidar_robosense.json -> nebula_ros_robosense/schema/
  - radar_continental.json -> nebula_ros_continental/schema/
  - Updated all schema file references to remove 'sub/' prefix
  - Added installation of common schema files to vendor packages so JSON schema references work
  - Updated documentation references to use new package paths
  * Flatten config directory structure: move configs from config/{lidar,radar}/{vendor}/ to config/
  - Moved all sensor config files directly into vendor package config/ directories
  - Updated launch file references to use new flattened paths
  - Updated nebula_examples launch file to reference nebula_ros_hesai instead of nebula_ros
  - Removed empty lidar/hesai, lidar/velodyne, lidar/robosense, and radar/continental subdirectories
  * Fix JSON schema validation using relative paths from schema file location
  - Added $id fields to common schema files using paths relative to repo root
  - Updated all $ref references to use relative paths from schema file location:
  - Common schemas: ../../nebula_ros_base/schema/<file>.json
  - Vendor schemas: <vendor_file>.json (same directory)
  - Validation works with --base-uri file://<repo_root>/<vendor>/schema/
  - All schema validations pass successfully for all vendor packages
  * temp: test updated json-schema-check
  * ci(pre-commit): autofix
  * temp: fix shortened commit hash
  * temp: update json schema check ref
  * temp: final json check version bump
  * fix(ars548): add missing type attr to diagnostics schema
  * docs: fix schema table gen
  * build: remove velodyne-specific stuff from base package
  * chore: remove accidentally committed sub-repos
  * refactor: group packages by vendor
  * refactor: move tests and examples to vendor pkgs, move nebula_launch to nebula meta package, remove nebula_sensor_driver meta package
  * ci(pre-commit): autofix
  * build: fixed cmake files
  * chore: clean up nebula_common
  * ci(pre-commit): autofix
  * move to src directory for cleanliness
  * build: remove superfluous dependencies in CMakeLists and package.xml
  * ci(pre-commit): autofix
  * build: fix missing PNG interface lib
  * build: fix agnocastlib not providing modern cmakle target
  * chore: rename too-convoluted nebula_ros_base_common to nebula_ros_base
  * docs: add migration guide to new version in readme
  * chore: version bump to 0.3.0
  * chore: rename nebula_ros to nebula
  * docs: updater readme to reflect nebula pkg rename
  * chore: rename nebula_ros_vendor to nebula_vendor
  * refactor: nebula_module_vendor -> nebula_vendor_module
  * test: fix issue where sourcing the built project was required for smoke tests to pass
  * chore: update docs, schemas to new module names
  * chore: revert unnecessary test_depend
  * chore: remove erroneous test method instructions
  * chore: remove now-unused hesai_decoder.hpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kotaro Uetake, Max Schmeller

0.2.15 (2025-10-29)
-------------------

0.2.14 (2025-10-22)
-------------------

0.2.13 (2025-10-01)
-------------------

0.2.12 (2025-08-25)
-------------------

0.2.11 (2025-08-22)
-------------------

0.2.10 (2025-08-14)
-------------------

0.2.9 (2025-07-23)
------------------

0.2.8 (2025-07-03)
------------------

0.2.7 (2025-06-09)
------------------

0.2.6 (2025-05-07)
------------------

0.2.5 (2025-03-26)
------------------

0.2.4 (2025-02-19)
------------------

0.2.3 (2025-02-04)
------------------

0.2.2 (2024-10-25)
------------------

0.2.1 (2024-10-03)
------------------

0.2.0 (2024-09-24)
------------------

0.0.1 (2024-09-11)
------------------
