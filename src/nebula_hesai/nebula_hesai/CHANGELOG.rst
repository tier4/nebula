^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nebula_hesai
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2026-07-30)
------------------
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
* fix(nebula_hesai): guard sensor setup with try/catch and retry (`#464 <https://github.com/tier4/nebula/issues/464>`_)
  * fix(nebula_hesai): remove throwaway thread+join in check_and_set_config
  The sensor setup path wrapped each blocking PTC command in a std::thread
  that was joined immediately. This added no concurrency, but it did change
  error semantics: an exception escaping a std::thread's entry function calls
  std::terminate() instead of propagating to the joining thread. So any
  recoverable comms fault during setup (e.g. get_lidar_range() returning
  "PTC Error: Timeout;" via value_or_throw) aborted the whole component
  container rather than being catchable by the caller.
  Inline all eight thread+join constructs so these exceptions propagate
  normally and can be handled by the caller. std::this_thread::sleep_for
  usage (and the <thread> include) is retained. Also drop two now-stale
  "waiting for thread"/"thread finished" debug logs.
  Pure refactor: no behavioral change on the success path. A follow-up will
  add try/catch + retry around check_and_set_config() so a transient setup
  timeout is non-fatal.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * fix(nebula_hesai): guard sensor setup with try/catch and retry
  Since PR `#463 <https://github.com/tier4/nebula/issues/463>`_ removed the throwaway threads from check_and_set_config, a
  comms fault during sensor setup now surfaces as a normal std::exception at
  the call site instead of calling std::terminate. But the call site did not
  handle it, so a transient timeout during startup would still propagate out
  of node construction (and out of the on_config_change parameter callback).
  Wrap the check_and_set_config() calls in a new configure_sensor() helper
  that catches std::exception and, when retry_hw is set, retries with the
  same 8s backoff already used for the TCP connect step. After a bounded
  number of failed attempts it logs and continues without applying the
  configuration rather than aborting, consistent with how the wrapper
  already tolerates get_inventory() failures and retry_hw=false connect
  failures.
  retry_hw is now stored as a member so both the constructor and
  on_config_change share the retry behaviour.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  * chore: reword partial reconfig error
  ---------
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
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
* chore: sync files (`#74 <https://github.com/tier4/nebula/issues/74>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
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
* Contributors: Koichi Imai, Max Schmeller, atsushi yano, tier4-nebula-app[bot]

1.1.1 (2026-06-03)
------------------

1.0.0 (2026-04-06)
------------------
* refactor(nebula_core_hw_interfaces): flatten obsolete include path (`#430 <https://github.com/tier4/nebula/issues/430>`_)
* Contributors: Max Schmeller

0.4.0 (2026-03-27)
------------------
* feat(nebula_continental, nebula_hesai): apply agnocast_components for CIE (`#418 <https://github.com/tier4/nebula/issues/418>`_)
  * feat(nebula_continental): apply autoware_agnocast_wrapper for CIE
  * ci(pre-commit): autofix
  * refactor(nebula_continental): replace autoware_agnocast_wrapper with agnocast_components
  * fix(nebula_continental): make agnocast_components dependency conditional on ENABLE_AGNOCAST
  Align with the Hesai package pattern by guarding the agnocast_components
  dependency behind the ENABLE_AGNOCAST environment variable, falling back
  to rclcpp_components_register_node when disabled. This fixes CI failures
  where rosdep cannot find ros-humble-agnocast-components.
  * fix(nebula_continental): register SRR520 with agnocast_components when ENABLE_AGNOCAST
  agnocast_components_register_node and rclcpp_components_register_node
  cannot coexist in the same package because both generate the same
  rclcpp_components resource index file via different mechanisms.
  Wrap SRR520 registration with the same ENABLE_AGNOCAST conditional.
  * feat(nebula_hesai): apply CIE with agnocast_components
  * docs: update README for CIE support and fix cmake message
  - Restructure Agnocast section with feature support table for Hesai/Continental
  - Add IPC zero-copy and CIE subsections
  - Fix cmake message() to use STATUS mode
  * ci(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  Co-authored-by: David Wong <33114676+drwnz@users.noreply.github.com>
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
* Contributors: David Wong, Max Schmeller, atsushi yano

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
* build: make `agnocastlib` an optional dependency (`#397 <https://github.com/tier4/nebula/issues/397>`_)
  * build: remove agnocastlib from package.xml if `ENABLE_AGNOCAST` env var is not `1`
  * ci: add agnocast/non-agnocast matrix builds
  * chore: run Codecov only for agnocast_enabled==0
  ---------
* Contributors: Max Schmeller

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
* fix: update calibration file paths in Hesai and Velodyne launch and schema files to reference the correct decoder packages (`#394 <https://github.com/tier4/nebula/issues/394>`_)
* test(hesai): regenerate out-of-date ground truhts for AT128 and XT16 (`#390 <https://github.com/tier4/nebula/issues/390>`_)
* chore(maintaners): update maintainers in all packages (`#385 <https://github.com/tier4/nebula/issues/385>`_)
  * chore(maintaners): update maintainers in all packages
  * chore(maintainers): add authors
  * chore(maintainers): add author to Continental packages
  ---------
* prevent node crash on LiDAR timeout (`#384 <https://github.com/tier4/nebula/issues/384>`_)
  * prevent node crash on LiDAR timeout
  * ci(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: David Wong, Max Schmeller, Taekjin LEE, kotaro-hihara

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
