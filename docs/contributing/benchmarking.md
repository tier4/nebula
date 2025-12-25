# Benchmarking

Nebula includes scripts for measuring decoder performance. Use these to validate that performance-related changes do not introduce regressions.

## Prerequisites

Install dependencies:

```shell
pip install -r scripts/requirements.txt
```

Additionally, `seaborn` is required for plotting:

```shell
pip install seaborn
```

## Running Benchmarks

Use `profiling_runner.bash` to run benchmarks:

```shell
./scripts/profiling_runner.bash <run_name> \
    --sensor-model <model> \
    --rosbag-path <path_to_rosbag>
```

### Required Arguments

| Argument | Description |
|----------|-------------|
| `<run_name>` | Name for this benchmark run (used in output filenames) |
| `--sensor-model`, `-m` | Sensor model name (e.g., `Pandar64`, `AT128`) |
| `--rosbag-path`, `-b` | Path to a rosbag containing sensor packets |

### Optional Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--runtime`, `-t` | 20 | Duration of each test run in seconds |
| `--n-runs`, `-n` | 3 | Number of runs per test |
| `--maxfreq`, `-f` | 2300000 | CPU frequency in Hz (for consistent results) |
| `--taskset-cores`, `-c` | all cores | CPU cores to pin the process to |

### Example

```shell
./scripts/profiling_runner.bash baseline \
    --sensor-model AT128 \
    --rosbag-path ~/rosbags/at128_sample \
    --n-runs 5 \
    --runtime 30
```

Output files are written to `profiling_output/`.

## Plotting Results

Use `plot_times.py` to visualize benchmark results:

```shell
./scripts/plot_times.py <run_name> [<run_name2> ...]
```

### Options

| Argument | Default | Description |
|----------|---------|-------------|
| `--metrics` | `receive decode publish` | Metrics to plot |

### Example

Compare two runs:

```shell
./scripts/plot_times.py baseline optimized --metrics decode publish
```

This displays boxen plots comparing the timing distributions.
