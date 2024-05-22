# %%

import argparse
import glob
import os
import re

from matplotlib import pyplot as plt
import pandas as pd


def parse_logs(run_name):
    dfs = []
    script_dir = os.path.dirname(os.path.abspath(__file__))
    glob_pattern = os.path.join(script_dir, f"../profiling_output/{run_name}-[0-9].log")
    for path in glob.glob(glob_pattern):
        with open(path) as f:
            lines = f.readlines()
            lines = filter(lambda line: "PROFILING" in line, lines)
            lines = [re.sub(r".*PROFILING (\{.*?\}).*", r"\1", line) for line in lines]
            records = [eval(line) for line in lines]

        dfs.append(pd.DataFrame(records))

    df = pd.concat(dfs)

    for col in [c for c in df.columns if c.startswith("d_")]:
        df[col] /= 1e6  # ns to ms
    return df


def plot_timing_comparison(run_names):
    scenario_dfs = {run_name: parse_logs(run_name) for run_name in run_names}

    n_cols = sum(1 for c in next(iter(scenario_dfs.values())).columns if c.startswith("d_"))

    fig, axs = plt.subplots(1 + n_cols, figsize=(10, 10), num="Timing comparison")
    ax_d = axs[0]
    ax_n = ax_d.twinx()
    ax_n.set_ylabel("# points published")
    boxes = axs[1:]

    for i, (label, df) in enumerate(scenario_dfs.items()):
        # durations = df['d_total']

        # ax_d.plot(durations.index, durations, label=label, linestyle='', marker='.')
        for col in filter(lambda col: col.startswith("n_"), df.columns):
            ax_n.plot(
                df.index, df[col], label=f"{label}::{col}", linestyle="", marker=".", color="black"
            )
        for col in filter(lambda col: col.startswith("d_"), df.columns):
            ax_d.plot(df.index, df[col], label=f"{label}::{col}", linestyle="", marker=".")

        d_columns = [col for col in df.columns if col.startswith("d_")]
        n_cols = len(d_columns)
        for j, col in enumerate(d_columns):
            if i == 0:
                boxes[j].set_ylabel(f"{col} (ms)")
            boxes[j].boxplot(df[col], positions=[i], labels=[label])

        ax_d.legend(loc="upper right")
        ax_d.set_ylabel("time (ms)")
        ax_d.set_xlabel("iteration")

    df_means = pd.DataFrame(
        {label: df.describe().loc["mean"] for label, df in scenario_dfs.items()}
    )

    df_stds = pd.DataFrame({label: df.describe().loc["std"] for label, df in scenario_dfs.items()})

    df_means = df_means.rename(index=lambda label: f"{label} AVG")
    df_stds = df_stds.rename(index=lambda label: f"{label} STD")

    df_stat = pd.concat([df_means, df_stds], axis=0)

    baseline_name = run_names[0]
    df_base = df_stat[baseline_name]
    for label in df_base.index:
        df_stat.loc[f"{label} % rel to {baseline_name}", :] = (
            df_stat.loc[label, :] / df_base.loc[label] * 100
        )

    print(df_stat.to_markdown())
    plt.show()


# %%

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "run_names",
        nargs="+",
        help="The names of the runs to plot, as entered in test_runner.bash (e.g. 'baseline', 'optimized')\nAll files of the form <run-name>-[0-9].log will be plotted.",
    )
    args = parser.parse_args()

    plot_timing_comparison(args.run_names)
