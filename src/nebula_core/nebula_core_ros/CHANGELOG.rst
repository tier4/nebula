^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nebula_core_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2026-07-30)
------------------
* chore: sync files (`#480 <https://github.com/tier4/nebula/issues/480>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(nebula_core_ros): make SyncToolingWorker compatible with agnocast_wrapper::Node (`#469 <https://github.com/tier4/nebula/issues/469>`_)
  * feat(nebula_core_ros): make shared diagnostics/sync helpers agnocast_wrapper::Node-compatible
  * feat(continental_ars548_ros_wrapper): migrate ContinentalARS548RosWrapper to agnocast_wrapper::Node (Method 2)
  * fix(continental_ars548_ros_wrapper): publish pass-through messages via publish(const &)
  The ALLOCATE_OUTPUT + *out = std::move(*msg) pattern stole msg's buffer, which
  was allocated on the normal heap (outside the borrow window), so the payload
  never entered shared memory and agnocast zero-copy delivered an invalid pointer
  to cross-process subscribers. publish(const &) borrows inside the window and
  copies into shared memory, which is correct on both the rclcpp and agnocast
  backends.
  * fix(nebula_core_ros): include renamed nebula_agnocast_wrapper.hpp in sync_tooling_worker
  * feat(nebula_hesai): migrate HesaiRosWrapper to agnocast_wrapper::Node (Method 2)
  * ci(pre-commit): autofix
  * fix(nebula_hesai): renamed wrapper include, const-ref packets publish, restore cie thread factory
  - include the post-`#472 <https://github.com/tier4/nebula/issues/472>`_ nebula_agnocast_wrapper.hpp (old autoware_agnocast_wrapper.hpp
  no longer exists)
  - publish raw packets via publish(*msg) so the payload is materialized into shared memory
  (a plain std::move would leave it on the normal heap, invalid for cross-process subscribers)
  - restore make_cie_thread_factory for the packets publish thread; the migration had dropped it,
  which would leave that non-ROS thread unmanaged by the Agnocast CIE thread configurator and
  lose its configured priority/affinity
  * fix(continental_ars548_ros_wrapper): log the resolved packets topic again
  The migration replaced `packets_sub\_->get_topic_name()` with the relative
  topic literal, which hides the actual (namespaced/remapped) topic and makes
  troubleshooting harder. The wrapper's Subscription has no get_topic_name(),
  so resolve the name through the node's topics interface instead; this yields
  the same fully-qualified name as before on both backends.
  * fix(nebula_hesai): log the resolved packets topic again
  The migration replaced `packets_sub\_->get_topic_name()` with the relative
  topic literal, which hides the actual (namespaced/remapped) topic and makes
  troubleshooting harder. The wrapper's Subscription has no get_topic_name(),
  so resolve the name through the node's topics interface instead; this yields
  the same fully-qualified name as before on both backends.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
* chore: sync files (`#475 <https://github.com/tier4/nebula/issues/475>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: sync files (`#473 <https://github.com/tier4/nebula/issues/473>`_)
  * chore: sync files
  * style(pre-commit): autofix
  * feat(agnocast_wrapper): adopt synced agnocast_wrapper core, retire hand-written wrapper (Stage 2) (`#467 <https://github.com/tier4/nebula/issues/467>`_)
  * feat(agnocast_wrapper): adopt synced agnocast_wrapper core, retire hand-written wrapper (Stage 2)
  Removes the hand-written nebula_agnocast_wrapper.hpp (Method-1) and switches the nebula_hesai decoder_wrapper to the synced umbrella's explicit-node API (NEBULA_CREATE_PUBLISHER2 -> NEBULA_CREATE_PUBLISHER2_ON_NODE, NEBULA_HAS_ANY_SUBSCRIPTIONS -> get_subscription_count() > 0). Nodes stay rclcpp::Node. The synced headers arrive via Stage 1 (`#466 <https://github.com/tier4/nebula/issues/466>`_) sync-files, so they are not in this diff.
  * refactor(nebula_hesai): use nebula_agnocast_wrapper.hpp include after `#472 <https://github.com/tier4/nebula/issues/472>`_ rename
  ---------
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
* ci(sync-files): name synced agnocast_wrapper umbrella nebula_agnocast_wrapper.hpp (`#472 <https://github.com/tier4/nebula/issues/472>`_)
  * ci(sync-files): name synced agnocast_wrapper umbrella nebula_agnocast_wrapper.hpp
  * ci(sync-files): remove orphaned autoware_agnocast_wrapper.hpp superseded by nebula\_ rename
  ---------
* chore: sync files (`#471 <https://github.com/tier4/nebula/issues/471>`_)
  * chore: sync files
  .
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: sync files (`#74 <https://github.com/tier4/nebula/issues/74>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(nebula_core_ros): templatize NodeT of `nebula_core_ros` (`#457 <https://github.com/tier4/nebula/issues/457>`_)
  * templatize NodeT of nebula_core_ros
  * ci(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(nebula_core, nebula_hesai): register hesai LiDAR packet threads with cie_thread_configurator (`#460 <https://github.com/tier4/nebula/issues/460>`_)
  * feat(nebula_core, nebula_hesai): register hesai LiDAR packet threads with cie_thread_configurator
  * fix: address review findings
  - add UdpSocket thread_factory unit test (TestThreadFactoryCreatesReceiverThread)
  - assert factory-created threads are joinable in UdpSocket and SingleConsumerProcessor
  - document the nullptr fallback behavior in the thread_factory param docs
  * refactor: document factory re-invocation on relaunch instead of testing it
  The subscribed-move path that re-invokes the thread factory is not
  exercised by any production code (all vendors subscribe only after the
  socket is emplaced), so pin it in the thread_factory_t docs rather than
  in the unit test. The test now covers only the factory-creates-thread
  contract.
  * fix: validate factory-created threads at runtime instead of assert
  Asserts are compiled out in release builds, so a factory returning a
  non-joinable thread would leave the processor or socket silently dead.
  SingleConsumerProcessor now throws std::invalid_argument (consistent
  with its other constructor validations); UdpSocket rolls back running\_
  and throws UsageError so the socket stays in a consistent
  unsubscribed state.
  * refactor: create all thread factories in the ROS wrapper layer
  Address review feedback: nebula_hesai_hw_interfaces must stay
  middleware-independent, including its build scripts. The Agnocast
  dependency and the USE_AGNOCAST_ENABLED conditional are now confined
  to the nebula_hesai ROS wrapper package:
  - Add a private cie_thread_factory.hpp helper in nebula_hesai that
  builds the Agnocast CIE thread factory (or nullptr when disabled).
  - Inject the UDP receive thread factory into HesaiHwInterface via a
  new optional constructor parameter instead of constructing it
  inside the HW interface.
  - Remove the Agnocast include, ifdef, CMake, and package.xml entries
  from nebula_hesai_hw_interfaces.
  * refactor: add StdThreadFactory to nebula_core_common and ban null thread factories
  - absorb thread_factory_t into nebula_core_common/util/thread_factory.hpp
  and default UdpSocket::subscribe() / SingleConsumerProcessor to
  StdThreadFactory instead of branching on nullptr internally
  - move the CIE thread factory helper from nebula_hesai to nebula_core_ros
  and return StdThreadFactory when Agnocast support is disabled
  - replace the fixed sleep in the UDP factory test with promise/future
  - add a test that the stored factory persists across unsubscribe,
  re-subscribe and move
  ---------
* Contributors: Koichi Imai, atsushi yano, tier4-nebula-app[bot]

1.1.1 (2026-06-03)
------------------
* fix(nebula_agnocast_wrapper): add agnocast intra process subscription count (`#447 <https://github.com/tier4/nebula/issues/447>`_)
  * fix: add intra_sub_count
  * chore: re-trigger CI
  * chore: re-trigger CI
  ---------
* Contributors: Yutaro Kobayashi

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
