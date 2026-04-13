^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nebula_sample_hw_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-04-06)
------------------
* refactor(nebula_core_hw_interfaces): flatten obsolete include path (`#430 <https://github.com/tier4/nebula/issues/430>`_)
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
* Contributors: David Wong, Max Schmeller

0.4.0 (2026-03-27)
------------------

0.3.2 (2026-01-27)
------------------

0.3.1 (2026-01-19)
------------------

0.3.0 (2025-12-02)
------------------

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
