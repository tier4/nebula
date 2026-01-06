#!/usr/bin/env python3
# %%

import argparse
import glob
import os

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


def plot_timing_comparison(run_names, metrics):
    """
    Plot comparison of timing metrics for a given set of runs.

    Loads metrics from CSV files in the profiling_output directory, and plots a boxen plot for
    each metric.

    Args:
        run_names: A list of run names to plot.
        metrics: A list of metrics to plot.
    Returns:
        None
    """
    all_data = []

    profiling_dir = "profiling_output"

    for run in run_names:
        for metric in metrics:
            # Try to find the matching file (format: <run-name>-<metric>-duration.csv)
            glob_pattern = os.path.join(profiling_dir, f"{run}-*-{metric}-duration.csv")
            matching = glob.glob(glob_pattern)
            if not matching:
                print(f"No matching file found for {glob_pattern}")
                continue
            for file in matching:
                series = pd.read_csv(file, header=None, names=["value"]).squeeze()
                for val in series:
                    all_data.append({"Run": run, "Metric": metric.capitalize(), "Value": val})

    df = pd.DataFrame(all_data)
    if df.empty:
        print("No data found for the specified runs/metrics.")
        return

    fig, axes = plt.subplots(1, len(metrics), figsize=(18, 6), sharey=False, squeeze=False)
    for ax, metric in zip(axes.flatten(), metrics):
        sns.boxenplot(data=df[df["Metric"] == metric.capitalize()], x="Run", y="Value", ax=ax)
        ax.set_title(f"{metric.capitalize()} Duration")
        ax.set_ylabel("Duration (ms)")
        ax.set_xlabel("Run")

    fig.tight_layout()
    plt.show()


# %%

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "run_names",
        nargs="+",
        help="The names of the runs to plot, as entered in test_runner.bash (e.g. 'baseline', 'optimized')\nAll files of the form <run-name>-[0-9].log will be plotted.",
    )
    parser.add_argument(
        "--metrics",
        nargs="+",
        help="The metrics to plot, any combination of 'receive', 'decode', 'publish'",
        default=["receive", "decode", "publish"],
        choices=["receive", "decode", "publish"],
    )
    args = parser.parse_args()

    plot_timing_comparison(args.run_names, args.metrics)
