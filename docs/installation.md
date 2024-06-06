# Installing Nebula

## Requirements

Nebula requires ROS 2 (Galactic or Humble) to build the ROS 2 wrapper.
Please see the [ROS 2 documentation](https://docs.ros.org/en/humble/index.html) for how to install.

## Getting the source and building

> **Note**
>
> A [TCP enabled version of ROS' Transport Driver](https://github.com/mojomex/transport_drivers/tree/mutable-buffer-in-udp-callback) is required to use Nebula.
> It is installed automatically into your workspace using the below commands. However, if you already have ROS transport driver binaries installed, you will have to uninstall them to avoid conflicts (replace `humble` with your ROS distribution):
> `sudo apt remove ros-humble-udp-driver ros-humble-io-context`

To build Nebula run the following commands in your workspace:

```bash
# In workspace
mkdir src
git clone https://github.com/tier4/nebula.git src
# Import dependencies
vcs import src < src/build_depends.repos
rosdep install --from-paths src --ignore-src -y -r
# Build Nebula
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
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
