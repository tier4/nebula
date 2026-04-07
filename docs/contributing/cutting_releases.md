---
icon: lucide/rocket
---

# Cutting Releases

Release rules at one glance:

* Releases are always cut from the `main` branch.
* Nebula follows [Semantic Versioning](https://semver.org/).
* Version tags are prefixed with `v`, e.g. `v1.2.3`.
* Nebula uses [Bloom and Catkin](https://docs.ros.org/en/humble/How-To-Guides/Releasing/Releasing-a-Package.html)
  to manage releases and changelogs.
* Releases and tags are never deleted or modified after creation. If a mistake is made, a new
  release should be created to fix it.

## Release Process

The below steps have to be followed strictly for every release:

1. Ensure all changes for the release are merged into `main`.
2. Follow the [Subsequent Releases](https://docs.ros.org/en/humble/How-To-Guides/Releasing/Subsequent-Releases.html)
   guide to update the changelog, bump the version, and create pull requests to rosdistro.
    1. Update changelogs using `catkin_generate_changelog`, then PR and merge to `main`.
    2. Bump the version using `catkin_prepare_release` directly on `main`.
    3. Notify the ROS 2 ecosystem using `bloom-release`. Do this once per supported ROS 2 distribution,
       e.g. `bloom-release --rosdistro humble nebula`, then `bloom-release --rosdistro jazzy nebula`.
3. Create a new GitHub release on the [GitHub Releases](https://github.com/tier4/nebula/releases/new)
   page, using the already created tag from step 2.

!!! warning "Check for breaking changes"
    As per the semantic versioning rules, the major/minor/patch version should be bumped based
    on the type of changes in the release. Familiarize with [Semantic Versioning](https://semver.org/),
    and ensure to bump the version correctly in accordance with the changes in the release.

## ROS 2 Releases

The above steps already cover the process for cutting ROS 2 releases, and these will
automatically be published as ROS 2 packages after some time. On the ROS side, the process looks
as follows:

1. You `bloom-release` a new version of the package.
    1. Bloom updates the [Nebula Release Repository](https://github.com/ros2-gbp/nebula-release)
    2. Bloom creates a pull request to [ros2/rosdistro](https://github.com/ros/rosdistro), e.g.
       [ros/rosdistro#50573](https://github.com/ros/rosdistro/pull/50573)
2. A rosdistro maintainer reviews and merges the pull request.
3. New packages appear in Rosdep, i.e. in `rosdep update && rosdep resolve my_new_package`.
4. The [ROS 2 buildfarm](https://build.ros2.org/) builds the new packages.
5. A ROS 2 [distro sync](https://discourse.openrobotics.org/tag/sync/117/l/latest) publishes
   the new packages to APT.

!!! warning "Rosdep-APT Time Delay"
    A package visible to Rosdep does not appear in APT until a ROS 2 distro sync happens, which
    is usually every 1-2 months. This means there is a significant time delay from cutting a
    release to users being able to install it via APT.
    Time-sensitive users should add Nebula to their `build_depends.repos` and do a source build.
