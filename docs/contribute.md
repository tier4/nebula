# Contributing to Nebula

Thank you for your interest in contributing to Nebula!
We’re excited to work with the open-source community to build an even better project.
Before submitting your contributions, please take a moment to review our guidelines to ensure a smooth process.

## Getting Started

- Set up your development environment: [installation](installation.md)
- Enable [pre-commit](https://pre-commit.com/#install)

  ```shell
  # Install pre-commit on your system
  pip3 install pre-commit

  # Enable pre-commit for Nebula
  cd ~/nebula_ws/src
  pre-commit install
  ```

- Set up your IDE to use Clang-Tidy with the `.clang-tidy` configuration found in the repository root
- Docs are found at [tier4.github.io/nebula](https://tier4.github.io/nebula)

## How to Contribute

### About Large Contributions

For large contributions, i.e. more than a few dozen lines or new features,
[create an issue](https://github.com/tier4/nebula/issues/new) first.
This allows us to give feedback early and allows for discussion about how/if the feature fits into
Nebula.

Once an issue is ready to be turned into a PR, make sure to comment on the issue that you are currently working on it.
This prevents double work from other community members.

Do not be afraid to open a draft PR in an early stage of your implementation.
This gives us a chance to spot potential problems early (e.g. line count) and lets us align with you better.

### What gets merged?

We are keen on merging PRs that are clear wins for the project:

- support for popular sensor models
- unit tests
- bug fixes
- robustness improvements, error handling and reporting

If a PR has value for the project but takes a long time or a lot of effort to review,
we will have to reject it. We will let you know if we think we can handle the scope of your
contribution in the discussion of your issue.

### What doesn't get merged?

- 500+ line PRs: split your PR into smaller ones
- PRs without sufficient unit tests: if unit tests are not rigorous enough or have low coverage
- PRs with multiple unrelated changes: split up into multiple PRs
- performance-heavy features: features with latency increases of more than a low single-digit percentage
- style changes
- big refactorings
- features implemented only for some vendors

## Pull Requests

Pull requests should be against the `develop` branch.
Hotfix PRs should be against the `master` branch.

A good PR fulfills all of the following:

- passes pre-commit
- conforms to the repo's `.clang-tidy` and `.clang-format`
- passes all CI checks
- a single clearly stated goal
- every line changed directly contributes to that goal
- unit tests for added/fixed functionality where applicable
- documentation for added functionality
- a reproducible evaluation / testing method where applicable
  - e.g. Rosbags or PCAPs attached
  - for performance optimizations, easily runnable benchmarks

We cannot start a review before pre-commit and CI are passing.

## Other Ways to Contribute

Bug reports in GitHub issues:

- provide logs, PCAPs and videos where possible
- specify the exact version and environment used
