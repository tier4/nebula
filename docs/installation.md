# Installing Nebula

## Requirements

Nebula officially supports the following ROS 2 distros:

- [Humble](https://docs.ros.org/en/humble/Installation.html)
- [Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

Other ROS 2 distros, such as [Rolling](https://docs.ros.org/en/rolling/Installation.html), might
work, but are not officially supported at this time.

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
vcs import < build_depends-${ROS_DISTRO}.repos
rosdep install --from-paths . --ignore-src -y -r
# Build Nebula
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Building only specific vendors

By default, building the `nebula` package will build all supported vendors. To build only
specific vendors, use the `--packages-up-to` flag:

```bash
# Build only Hesai support
colcon build --packages-up-to nebula_hesai

# Build multiple vendors
colcon build --packages-up-to nebula_hesai nebula_velodyne
```

Available vendor packages are:

- `nebula_hesai` - Hesai LiDARs (Pandar series, AT128, OT128, etc.)
- `nebula_velodyne` - Velodyne LiDARs (VLP-16, VLP-32, VLS-128)
- `nebula_robosense` - Robosense LiDARs (Bpearl, Helios)
- `nebula_continental` - Continental radars (ARS548, SRR520)

## Testing your build

To run Nebula unit tests:

```bash
colcon test
```

Show results:

```bash
colcon test-result --all
```
