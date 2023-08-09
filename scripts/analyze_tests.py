import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

log_path = "../../log/latest_test/events.log"

with open(log_path, 'r') as f:
    text = f.read()

#data_regex = r"64.cpp.*?\n.*?Expected.*?\n.*?p1.*?\n.*?Which is: ([+-0-9.e]+).*?\n.*?p2.*?\n.*?Which is: ([+-0-9.e]+)"
data_regex = r"'line': b'([0-9]+):.*?<<<([-0-9.e]+), ([-0-9.e]+), ([-0-9.e]+), ([-0-9.e]+)>>>"
data = re.findall(data_regex, text)
#data = [x for x in data if x[0] != '-' and x[1] != '-']
data = [(int(i), float(x), float(y), float(z), float(w)) for i, x, y, z, w in data]

print(f"Found {len(data)} data points")
if len(data) == 0:
    exit(1)

df = pd.DataFrame(data, columns=['run', 'azi_ref', 'azi', 'd_ref', 'd'])

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
    for col in df.columns:
        if col == 'run':
            continue

        color = next(cycle)['color']
        if "azi" in col:
            ax.plot(df[df['run'] == r][col], label=col, color=color)
        else:
            ax2.plot(df[df['run'] == r][col], label=col, color=color)
plt.show()