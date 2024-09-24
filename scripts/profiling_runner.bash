#!/usr/bin/bash

# Configuration

print_usage() {
    # Print optionally passed error message
    if [ -n "$1" ]; then
        echo "$1"
    fi
    echo "Usage: $0 <run_name> [-t|--timeout <timeout>] [-m|--sensor-model <sensor_model>] [-n|--n-runs <n_runs>] [-f|--maxfreq <maxfreq>] [-c|--taskset-cores <core1[,core2,...]>] [-b|--rosbag-path <rosbag_path>]"
    echo "  -t|--runtime:       runtime for each test run (default: $runtime s)"
    echo "  -m|--sensor-model:  the model name of the sensor to evaluate Nebula on (e.g. 'Pandar64')"
    echo "  -n|--n-runs:        number of runs per test (default: $n_runs)"
    echo "  -f|--maxfreq:       frequency to set all CPU cores to (default: $maxfreq Hz)"
    echo "  -c|--taskset-cores: cores to run Nebula on. Ideally isolated cores. (default: $taskset_cores)"
    echo "  -b|--rosbag-path:   path to rosbag file"
}

# Default parameters
runtime=20                       # seconds (rosbag runtime is timeout-3s for startup)
n_runs=3                         # number of runs per test
maxfreq=2300000                  # 2.3 GHz
n_cores=$(nproc --all)           # number of cores of the CPU
taskset_cores=0-$((n_cores - 1)) # cores to run Nebula on
nebula_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." &>/dev/null && pwd)

if [ $# -lt 1 ]; then
    print_usage "No run name specified"
    exit 1
fi

run_name=$1
shift

#do the parsing
while [[ $# -gt 0 ]]; do
    key="$1"

    case $key in
    -t | --runtime)
        runtime="$2"
        shift
        shift
        ;;
    -m | --sensor-model)
        sensor_model="$2"
        shift
        shift
        ;;
    -n | --n-runs)
        n_runs="$2"
        shift
        shift
        ;;
    -f | --maxfreq)
        maxfreq="$2"
        shift
        shift
        ;;
    -c | --taskset-cores)
        taskset_cores="$2"
        shift
        shift
        ;;
    -d | --nebula-dir)
        nebula_dir="$2"
        shift
        shift
        ;;
    -b | --rosbag-path)
        rosbag_path="$2"
        shift
        shift
        ;;
    *)
        print_usage "Unknown option $1"
        exit 1
        ;;
    esac
done
# Enf of configuration

if [ -z "$rosbag_path" ]; then
    print_usage "No --rosbag-path specified"
    exit 1
fi

if [ -z "$sensor_model" ]; then
    print_usage "No --sensor-model specified"
    exit 1
fi

echo "Nebula home is $nebula_dir"
output_dir="$nebula_dir/profiling_output"
mkdir -p "$output_dir"
echo "Outputting to '$output_dir/$run_name-[1-$n_runs].log'"

cd "$nebula_dir" || exit

lockfreq() {
    for ((i = 0; i < n_cores; i++)); do
        echo userspace | sudo tee /sys/devices/system/cpu/cpufreq/policy$i/scaling_governor >/dev/null
        echo "$maxfreq" | sudo tee /sys/devices/system/cpu/cpufreq/policy$i/scaling_setspeed >/dev/null
        echo -n "CPU $i's frequency is: "
        sudo cat /sys/devices/system/cpu/cpu$i/cpufreq/cpuinfo_cur_freq
    done

    sudo sh -c "echo 0 > /sys/devices/system/cpu/cpufreq/boost"
}

unlockfreq() {
    for ((i = 0; i < n_cores; i++)); do
        echo schedutil | sudo tee /sys/devices/system/cpu/cpufreq/policy$i/scaling_governor >/dev/null
        echo -n "CPU $i's frequency is: "
        sudo cat /sys/devices/system/cpu/cpu$i/cpufreq/cpuinfo_cur_freq
    done

    sudo sh -c "echo 1 > /sys/devices/system/cpu/cpufreq/boost"
}

echo "Setting up for compiling"
unlockfreq

colcon build --symlink-install --packages-up-to nebula_ros --cmake-args -DCMAKE_BUILD_TYPE=Release || exit 1
# shellcheck disable=SC1091
source "$nebula_dir/install/setup.bash"

ros2 daemon stop
ros2 daemon start

echo "Setting up for test run"
lockfreq

for ((i = 1; i <= n_runs; i++)); do
    echo "Running iteration $i of $n_runs"
    # set the log level to debug for all ros 2 nodes

    timeout -s INT "$runtime" taskset -c "$taskset_cores" ros2 launch nebula_ros nebula_launch.py "sensor_model:=$sensor_model" launch_hw:=false debug_logging:=true >"$output_dir/$run_name-$i.log" 2>&1 &
    (sleep 3 && timeout -s KILL $((runtime - 3)) nohup ros2 bag play -l "$rosbag_path") >/dev/null &
    wait
done

echo "Unlocking frequency"
unlockfreq
