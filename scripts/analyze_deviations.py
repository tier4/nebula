#!/usr/bin/python3

import os
import re
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from collections import defaultdict
from typing import DefaultDict, List
from tqdm import tqdm

log_path = f"{os.path.dirname(__file__)}/../../log/latest_test/events.log"

def parse(path):
    with open(path, "r") as f:
        lines = f.readlines()

    pattern = r"^.*?\{'line': b['\"](.*)['\"]\}$"
    lines = [re.sub(pattern, r"\1", l) for l in tqdm(lines, desc="Stripping lines") if re.search(pattern, l)]
    pattern = r"([0-9]+): (.*?)(?:\\n)?$"
    matches = [re.match(pattern, l) for l in tqdm(lines, desc="Matching content of lines")]
    matches = [(m.group(1), m.group(2)) for m in matches if m]

    data: DefaultDict[str, List[str]] = defaultdict(list)
    for run, line in tqdm(matches, desc="Appending matches"):
        data[run].append(line)

    data_conv = {}
    for run in data.keys():
        text = "".join(data[run])
        records = re.findall(r"\{[a-z0-9+\-_:', \.]+\}", text)
        records = [r for r in records if r]
        records = eval("[" + ", ".join(records) + "]")
        if not records:
            continue
        data_conv[run] = pd.DataFrame(records)
        
    return data_conv

dfs = parse(log_path)

run_names = {
    1: "AT128",
    2: "XT32M",
    3: "40P",
    4: "64",
    5: "QT64",
    6: "XT32"
}

def plot(df_dict):
  fig, axs = plt.subplots(len(df_dict.keys()))

  for i, (run, df) in enumerate(df_dict.items()):
    ax = axs[i]
    ax.set_title(f"Distance vs. Relative Error -- Pandar{run_names[int(run)]}")
    ax.scatter(np.abs(df['d']), df['d_diff'] / np.abs(df['d']) * 1000, marker=".")
    ax.set_xlabel("Distance [m]")
    ax.set_ylabel("Relative Error [mm/m]")

  plt.show()
          
plot(dfs)

