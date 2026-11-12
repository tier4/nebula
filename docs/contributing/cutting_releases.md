---
icon: lucide/rocket
---

<!-- cspell:ignore rosrpm -->
<!-- prettier-ignore-start -->

# Cutting Releases

Release rules at one glance:

* Releases are always cut from the `main` branch.
* Nebula follows [Semantic Versioning](https://semver.org/).
* Version tags are prefixed with `v`, e.g. `v1.2.3`.
* The tag `vX.Y.Z` must point at a commit where every `package.xml` has version `X.Y.Z`. Bloom
  reads the release version from the `package.xml` files at that tag.
* Nebula uses [Bloom and Catkin](https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html)
  to manage releases and changelogs.
* Releases and tags are never deleted or modified after creation. GitHub enforces this for tags.
  If a mistake is made, a new release should be created to fix it.

## Prerequisites

* Write access to `tier4/nebula`.
* For `bloom-release`: membership in the `tier4` team defined in
  [ros2-gbp-github-org/tier4.tf](https://github.com/ros2-gbp/ros2-gbp-github-org/blob/latest/tier4.tf),
  which grants push access to the [Nebula Release Repository](https://github.com/ros2-gbp/nebula-release).
  Request it with a pull request to that file.
* `python3-bloom` >= 0.14 and `python3-rosdistro` >= 1.1.0 from APT, and a GitHub token for Bloom
  (it asks for one on the first run).
* A regular clone of `tier4/nebula`. `catkin_prepare_release` does not work in a git worktree.

## Release Process

`main` is protected, so unlike in the upstream
[Subsequent Releases](https://docs.ros.org/en/humble/How-To-Guides/Releasing/Subsequent-Releases.html)
guide, the version bump goes through a pull request. The below steps have to be followed strictly
for every release:

1. Ensure all changes for the release are merged into `main`.
2. Create the release pull request:

    ```shell
    git fetch origin --tags
    git switch -c chore/release-vX.Y.Z origin/main
    catkin_generate_changelog
    # Condense the "Forthcoming" sections to one entry per user-facing change
    git add src && git commit -m "docs: update changelogs"
    catkin_prepare_release --version X.Y.Z -t v --no-push -y
    git tag -d vX.Y.Z  # the real tag is created after merging (step 3)
    git push -u origin HEAD
    gh pr create --title "chore: release vX.Y.Z"
    ```

    Get it reviewed and **squash-merge** it. Do not name the branch `release/...`, as that pattern
    only accepts changes through pull requests.
3. Tag the squash commit and publish the GitHub release in one step:

    ```shell
    gh release create vX.Y.Z --target <squash commit SHA> --generate-notes --notes "<one-sentence summary>"
    ```

    Tags cannot be moved or deleted, so double-check the SHA.
4. Release to the ROS 2 buildfarm, once per supported ROS 2 distribution:

    ```shell
    bloom-release --rosdistro humble nebula
    bloom-release --rosdistro jazzy nebula
    ```

    Each run asks once whether to skip a failed `rosrpm` (RHEL) generator. Answer `y`: Nebula
    depends on `libpng++-dev`, which has no RHEL package. Abort on any other prompt.
5. If the release fixes [security advisories](https://github.com/tier4/nebula/security/advisories),
   set the patched version on each of them.

!!! warning "Check for breaking changes"
    As per the semantic versioning rules, the major/minor/patch version should be bumped based
    on the type of changes in the release. Please familiarize yourself with [Semantic Versioning](https://semver.org/),
    and ensure the version is bumped correctly in accordance with the changes in the release.

## ROS 2 Releases

The above steps already cover the process for cutting ROS 2 releases, and these will
automatically be published as ROS 2 packages after some time. On the ROS side, the process looks
as follows:

1. You `bloom-release` a new version of the package.
    1. Bloom updates the [Nebula Release Repository](https://github.com/ros2-gbp/nebula-release)
    2. Bloom creates a pull request to [ros/rosdistro](https://github.com/ros/rosdistro), e.g.
       [ros/rosdistro#50573](https://github.com/ros/rosdistro/pull/50573)
2. A rosdistro maintainer reviews and merges the pull request.
3. Packages added in this release become known to Rosdep, i.e. `rosdep update && rosdep resolve my_new_package`
   works. Rosdep maps package names, not versions, so existing packages are unaffected.
4. The [ROS 2 buildfarm](https://build.ros2.org/) builds the new packages and publishes them to the
   `ros2-testing` APT repository.
5. A ROS 2 [distro sync](https://discourse.openrobotics.org/tag/sync/117/l/latest) publishes
   the new packages to the main APT repository.

!!! warning "Rosdep-APT Time Delay"
    A new version does not appear in the main APT repository until the next ROS 2 distro sync.
    This means there is a significant time delay from cutting a release to users being able to
    install it via APT.
    Time-sensitive users can install from the `ros2-testing` repository (see
    [Testing with Pre-release Binaries](https://docs.ros.org/en/humble/Installation/Testing.html)),
    or add Nebula to their `build_depends.repos` and do a source build.

<!-- prettier-ignore-end -->
