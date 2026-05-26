^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nebula_robosense
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(nebula_core): remove transport drivers dependency, replace with local implementations (`#386 <https://github.com/tier4/nebula/issues/386>`_)
  * feat(nebula_core): tcp and can interfaces
  * feature(nebula_core): replace transport drivers tcp and can with local implementations
  * fix(nebula_core): address review comments
  * fix(nebula_core): handling potential race condition in http client
  * style(velodyne): remove dead code from http request logic
  * refactor(nebula_core): unify CAN socket errors with socket_utils.hpp
  * fix(nebula_core): throw exception for partial CAN FD reads instead of silent failure
  * docs(nebula_core): document chunked encoding limitation in HttpClient
  * style(nebula_core): remove outdated comment in tcp.hpp
  * style(robosense): remove unused array include
  * ci(pre-commit): autofix
  * fix(nebula_core): compilation error for CAN unit test
  * fix(nebula_core): update the build dependencies
  * chore(nebula_core): cspell for canfd
  * fix(hesai_hw_interface): re-implement missing functions
  * fix(continental_hw_interfaces): re-implement missing functions and bus time handling
  * refactor(connections): refactored common connection logic into socket_utils
  * fix(hesai): improve robust initialization and clean comments
  * perf(robosense): optimize packet callback to use swap data to avoid copy
  * ci(pre-commit): autofix
  * feat(core): chunked encoding support and connection docs
  * fix(hesai): non-const vector references
  * refactor(continental): remove uneccesary rclcpp dependencies
  * test(core): update unit tests and increase coverage
  * feat(nebula_core_hw_interfaces): robustness and timestamp fixes for sockets
  - Catch unhandled SocketError exceptions in TCP/UDP receiver threads
  - Enable hardware timestamping in async CAN receiver by using recvmsg
  - Update CanSocket callback signature to include RxMetadata
  - Add unit test for async CAN timestamp verification
  * refactor(nebula_core_hw_interfaces): socket cleanup and code quality improvements
  * ci(pre-commit): autofix
  * refactor(nebula_core_hw_interfaces): move CAN filter parsing to can.hpp
  * refactor(nebula): rename driver to socket in members and methods
  * fix(nebula_continental): restore missing dependencies and fix header extensions
  * refactor(nebula): fix build dependencies and remove redundant wrapper
  * refactor(nebula_core_hw_interfaces): remove close() from socket classes
  * refactor(nebula_core_hw_interfaces): enforce strict RAII for TcpSocket
  * refactor(connections): enforce best practices for Sockets
  - refactor(socket_utils): return expected from to_string
  - feat(tcp): add timeout to receive
  - refactor(tcp): enforce strict error handling (throw on close)
  * fix(tcp): handle partial sends and fix receive() docstring
  * fix(udp): add port check to sender filter validation
  * fix(http_client): prevent use-after-free by unsubscribing before return
  * fix(connections): restructure move constructors to avoid UB
  * fix(socket_utils): handle EINTR in is_socket_ready()
  * refactor(can): log warning on invalid filter parsing
  * fix(can): enable proper CAN FD support and improve error handling
  * refactor(http_client): fix O(N^2) parsing, header case-sensitivity, and connection logic
  * refactor(socket_utils): fix polling bug and modernize socket handling
  * refactor(tcp): fix stability bugs (EINTR, SIGPIPE), concurrency safety, and callback performance
  * refactor(udp): stability (EINTR), optimize hot loop (allocation/construction), and metrics
  * refactor(http): robust parsing, chunk overflow safety, and connection close latency fix
  * test(socket): add invalid IP handling tests for TcpSocket and UdpSocket
  * test(socket): add invalid input tests for HttpClient and CanSocket
  * test(socket): add Endpoint conversion and error handling tests for socket_utils
  * test(socket): add Endpoint conversion and error handling tests for socket_utils
  * fix(can): handle EINTR in recvmsg and cleanup comments
  * fix(tcp): cleanup error handling comments
  * style(can): add cspell ignore for TIMESTAMPNS
  * style(can): apply clang-format
  * feat(hesai): restore missing HTTP implementations for PTP and Sync Angle
  * fix(nebula_continental): correct message_filters header extensions
  * fix(nebula_core_hw_interfaces): add bounds checking for CAN FD frame reception
  * fix(nebula_core_hw_interfaces): improve CAN FD frame validation and type safety
  * fix(nebula_core_hw_interfaces): increase socket robustness to prevent TCP hanging on initialization, improve error handling
  * test(nebula_core_hw_interfaces): test edge-cases for socket connections, and add mock server for CAN unit testing
  * test(hw_interfaces): add unit tests for new socket implementations in each sensor hw interface
  * fix(hesai): reset tcp socket on ptc protocol errors to prevent desynchronization
  * fix(hesai): correct HTTP commands
  * build(ars548): remove message_filters as they're not used in the code
  * build: also remove deps from jazzy repos
  * chore(udp): trim implementation changes to bare minimum
  * ci(pre-commit): autofix
  * chore(udp): remove unused poll_fd\_
  * chore(socket_utils): small lint fixes
  * fix(socket_utils): revert default events in is_socket_ready that can cause recvmsg to fail
  * chore(tcp): move receive function to more visible spot
  * chore(tcp): lint cleanups, remove catch-all try block
  * refactor(velodyne_hw_interface): remove unused HTTP client driver method and include
  * fix(tcp): avoid throwing exceptions in receive thread
  * chore: small test_socket_utils tweaks
  * chore(tcp): remove async API due to tedious error handling
  * test(tcp): remove async TCP related tests
  * refactor(http): simplify by using synchronous TCP API
  * chore(hesai_hw_interface): catch HTTP errors, log error, and return status code
  * test(http_client): fix exception handling validation in error condition tests
  * chore: update copyright year to 2026 for new files
  * test(http_client): apply review suggestions
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Max SCHMELLER <max.schmeller@tier4.jp>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
* Contributors: David Wong, Max Schmeller

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
