# Installing Nebula

## Requirements

Nebula requires ROS 2 (Galactic or Humble) to build the ROS 2 wrapper.
Please see the [ROS 2 documentation](https://docs.ros.org/en/humble/index.html) for how to install.

## Getting the source and building

> **Note**
>
> Boost version 1.74.0 or later is required. A manual install may be required in Ubuntu versions earlier than 22.04.

To build Nebula run the following commands in your workspace:

```bash
# In workspace
git clone https://github.com/tier4/nebula.git
cd nebula
# Import dependencies
vcs import < build_depends.repos
rosdep install --from-paths . --ignore-src -y -r
# Build Nebula
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Testing your build

To run Nebula unit tests:

```bash
colcon test --event-handlers console_cohesion+ --packages-above nebula_common
```

Show results:

```bash
colcon test-result --all
```
