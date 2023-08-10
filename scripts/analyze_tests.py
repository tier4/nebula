import re
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

log_path = f"{os.path.dirname(__file__)}/../../log/latest_test/events.log"

with open(log_path, 'r') as f:
    text = f.read()

#data_regex = r"64.cpp.*?\n.*?Expected.*?\n.*?p1.*?\n.*?Which is: ([+-0-9.e]+).*?\n.*?p2.*?\n.*?Which is: ([+-0-9.e]+)"
data_regex = r"'line': b'([0-9]+):.*?<<<([-0-9.e]+), ([-0-9.e]+)>>>"
data = re.findall(data_regex, text)
#data = [x for x in data if x[0] != '-' and x[1] != '-']
data = [(int(i), float(x), float(y)) for i, x, y in data]

print(f"Found {len(data)} data points")
if len(data) == 0:
    exit(1)

df = pd.DataFrame(data, columns=['run', 'azi', 'azi_ref'])

run_names = {
    1: "AT128",
    2: "XT32M",
    3: "40P",
    4: "64",
    5: "QT64",
    6: "XT32"
}


cycler = plt.rcParams['axes.prop_cycle']
unique_runs = sorted(df['run'].unique())
fig, axs = plt.subplots(len(unique_runs))

for r, ax in zip(unique_runs, axs):
    cycle = ax._get_lines.prop_cycler
    ax2 = ax.twinx()
    ax.set_title(run_names[r])
    df_run = df[df['run'] == r]
    df_run['idx'] = range(len(df_run))
    df_run = df_run.set_index('idx')
    del df_run['run']
    for col in df.columns:
        if col == 'run':
            continue

        if col == 'azi_ref':
            df_diff = df_run[np.abs(df_run['azi'] - df_run['azi_ref']) > .008]
            ax.scatter(df_diff.index, df_diff['azi'], label=col, color='red')

        color = next(cycle)['color']
        if "azi" in col:
            ax.plot(df_run[col], label=col, color=color)
        else:
            ax2.plot(df_run[col], label=col, color=color)
plt.show()