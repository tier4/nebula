#!/usr/bin/python3

import argparse
from collections import defaultdict
import json
import os
import re

from matplotlib import pyplot as plt
import pandas as pd


def condition_data(log_file: str):
    with open(log_file, "r") as f:
        lines = f.readlines()
        lines = [re.search(r'(\{"type":.*?"tag":.*?\})', line) for line in lines]
        lines = [re.sub(r'([0-9])"', r"\1", line[1]) for line in lines if line]
        lines = [json.loads(line) for line in lines if line]

        cols = defaultdict(list)

        for record in lines:
            for key in record.keys():
                if key in ["tag", "type"]:
                    continue

                colname = f"{record['type']}.{key}.{record['tag']}"
                cols[colname] += [num for num in record[key] if num is not None]

    def quantile_filter(series, quantile):
        q = series.quantile(quantile)
        return series[series <= q]

    cols = {k: pd.Series(v, name=k) / 1e3 for k, v in cols.items()}
    cols = {k: quantile_filter(v, 0.999) for k, v in cols.items()}

    for v in cols.values():
        v: pd.Series
        v.attrs["file"] = os.path.basename(os.path.splitext(log_file)[0])

    return cols


def plot(conditioned_logs):

    conditioned_logs = {k: [dic[k] for dic in conditioned_logs] for k in conditioned_logs[0]}

    fig, axs = plt.subplots(1, len(conditioned_logs), figsize=(15, 8), dpi=120)

    handles, labels = [], []

    for i, k in enumerate(sorted(conditioned_logs.keys())):
        k: str
        v = conditioned_logs[k]
        ax: plt.Axes = axs[i]
        art = ax.boxplot(v)
        handles.append(art)
        labels.append(k)
        ax.set_ylabel("dt [µs]")
        ax.set_title(k.replace(".", "\n"))
        ax.set_xticks([1 + i for i in range(len(v))], [ser.attrs["file"] for ser in v])
        ax.tick_params(axis="x", labelrotation=90)

    fig.tight_layout()
    plt.savefig("instrumentation.png")
    plt.show()


def main(args):
    conditioned_logs = [condition_data(f) for f in args.log_files]
    plot(conditioned_logs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("log_files", nargs="+")
    args = parser.parse_args()

    main(args)
