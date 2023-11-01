import argparse
import json
import re
from typing import Iterable, List
from matplotlib.colors import LogNorm
import pandas as pd
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser(description='Analyze angle correction')
parser.add_argument('log_files', nargs='+')

args = parser.parse_args()

dfs = []
for log_file in args.log_files:
    with open(log_file, 'r') as f:
        lines = f.readlines()

    matches = [re.search(r'\[ANGDBG\]\s+(\{.*?\})', line) for line in lines]
    matches = [m for m in matches if m is not None]
    data = [json.loads(m.group(1)) for m in matches]
    df = pd.DataFrame(data)
    df.sort_values("out_deg", inplace=True)
    t_max = df['t_packet'].max()
    df = df[(t_max - df['t_packet']) <= 2e8]
    df['tos'] = df['t_packet'] % 1e8
    dfs.append(df)

fig, axs = plt.subplots(2, len(dfs), num='Angle correction', gridspec_kw={'height_ratios': [.9, .1]})
axs: Iterable[plt.Axes]

ax0_r = None

for ax, df, log_file, cax in zip(axs[0], dfs, args.log_files, axs[1]):
    ax: plt.Axes

    ax.sharey(axs[0][0])

    sc = ax.scatter(df['raw_deg'], df['out_deg'], c=df['tos'], marker='o', norm=LogNorm(vmin=1e6, vmax=1e8))
    ax.axvline(df['scan_phase'].iloc[0], color='black')
    ax.axhline(int(log_file.split('phase')[1].split('.')[0]), color='black')
    ax.set_xlabel('Raw angle (deg)')
    ax.set_ylabel('Out angle (deg)')
    ax.set_title(log_file)
    cax: plt.Axes
    fig.colorbar(sc, cax=cax, orientation='horizontal')

plt.show()
