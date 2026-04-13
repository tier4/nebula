^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nebula_core_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-04-06)
------------------
* feat(nebula_sample): add sample sensor package as template (`#382 <https://github.com/tier4/nebula/issues/382>`_)
  * feat(nebula_sample): add sample sensor package as template
  * feat(nebula_sample): add integration guide
  * feat(nebula_sample): update license year
  * chore(nebula_sample): cspell fixes and integration guide clarity
  * feat(sample_sensor): remove calibration configuration
  * feat(sample_sensor): package and CMakeLists tidy up
  * ci(pre-commit): autofix
  * docs(nebula_sample): document implementation requirements and add vendor-neutral guidance
  * docs(nebula_sample): add integration guide to mkdocs
  * ci(pre-commit): allow unsafe yaml tags for mkdocs
  * docs(sample_sensor): remove duplicate integration guide, modify links and titles
  * feat(sample_sensor): package and integration guide minor fixes
  * chore(nebula_sample): fix capitalization in integration guide entry
  * docs(sample_sensor): make integration guide diagrams visible in dark mode
  * docs(sample_sensor): remove commented code
  * docs(sample_sensor): simplify diagrams
  * ci(pre-commit): autofix
  * docs(sample_sensor): simplify integration guide
  * docs(integration_guide): delete unnecessary stylesheets
  * docs: fix mermaid and drawio errors
  * docs: fix checklist
  * chore: remove unsafe argument from check-yaml hook
  * docs: remove manual table of contents
  * docs(api): restore C++ API docs for sample
  * docs(integration): drop redundant divider
  * docs(integration): align architecture with current driver/wrapper structure
  * docs(integration): adjust rename checklist to match current patterns
  * docs(integration): use real UdpSocket API via snippets
  * docs(integration): recommend util::expected for errors
  * docs(integration): clarify azimuth wraparound handling
  * docs(integration): document angle normalization utility
  * docs(integration): explain logger dependency injection
  * docs(integration): showcase RateBoundStatus and LivenessMonitor
  * docs(integration): drop heading numbering
  * docs(integration): render wrapper example via snippets
  * docs(nav): move integration guide under contributing
  * docs(integration): avoid connection-loss log spam
  * docs(integration): require transparent recovery on disconnect
  * docs(integration): emphasize RAII for shutdown
  * docs(integration): focus diagnostics on core helpers
  * docs(integration): correct offline test to rosbag2
  * docs: add pymdown-extensions for tasklists
  * docs(integration): remove horizontal rules
  * docs(integration): make startup exceptions optional
  * chore: remove nebula_common directory
  * docs: make integration guide more concise and informative
  * chore: add example cpps to use as snippets in integ guide
  * chore: better expected docs
  * chore: nicer sample_common
  * chore: streamlining changes
  * refactor(nebula_sample): align wrapper wiring and typed error flow
  * refactor(nebula_sample_common): type calibration expected errors
  * refactor(nebula_sample_decoders): streamline decoder stub and error reporting
  * refactor(nebula_sample_decoders): clarify decode errors and add dummy scan completion
  * refactor(core): move angle utilities to nebula_core_common
  * docs: update angle utility snippet paths after core move
  * fix: address Codex PR review comments
  * docs: improve formatting and clarity in integration guide
  * refactor: enhance documentation and structure in sample ROS wrapper and decoder interfaces
  * docs: enhance integration guide with clearer implementation details and order of operations
  * chore: address codex review feedback
  * feat: add diagnostics and error handling to SampleRosWrapper and SampleHwInterface
  * feat: Make different launch_hw modes mutually explicit
  * feat: update topic names for point cloud and packet publishing in SampleRosWrapper
  * docs(sample_sensor): typo and add markdown links
  * refactor(nebula_sample): minimalist cleanup of core and vendor packages for closeness to main branch
  * docs(nebula_core): fix minor API inconsistency in subscriber, referenced in integration guide
  * fix(sample): restore sensor IP filtering and simplify template stubs
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  Co-authored-by: Max SCHMELLER <max.schmeller@tier4.jp>
* Contributors: David Wong

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
* feat: jazzy support (`#398 <https://github.com/tier4/nebula/issues/398>`_)
  * chore: split build_depends into common file and distro-specific ones
  * chore: ci only supports one build_depends file, so absorb the common one into each distro
  * ci: run build-and-test-differential as humble,jazzy matrix
  * ci: run build-and-test as humble,jazzy matrix
  * chore: only upload codecov once
  * docs: remove ignore-errors flag from rosdep
  * build: allow agnocast dep only on Humble
  * chore: remove unused ament_index_cpp header
  * fix: rewrite velodyne calib code to modern yaml-cpp to prevent dangling pointer error
  * chore: remove unused ament header for all vendors
  * chore: add compatibility code for serialized_bag_message that works for humble and jazzy
  * chore: use serialized_bag_message compat functions everywhere a bag_message->time_stamp was being read
  * fix(jazzy): updated removed timezone names for Ubuntu 24.04
  * ci: exclude invalid agnocast/jazzy matrix combo
  * docs: update mentions of ros distros
  * chore: use `auto` in explicit conversions
  ---------
* Contributors: Max Schmeller

0.3.1 (2026-01-19)
------------------
* chore(maintaners): update maintainers in all packages (`#385 <https://github.com/tier4/nebula/issues/385>`_)
  * chore(maintaners): update maintainers in all packages
  * chore(maintainers): add authors
  * chore(maintainers): add author to Continental packages
  ---------
* Contributors: David Wong

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
