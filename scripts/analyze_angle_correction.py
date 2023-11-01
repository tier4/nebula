import argparse
import json
import re
import pandas as pd
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser(description='Analyze angle correction')
parser.add_argument('log_file')

args = parser.parse_args()

with open(args.log_file, 'r') as f:
    lines = f.readlines()

matches = [re.search(r'\[ANGDBG\]\s+(\{.*?\})', line) for line in lines]
matches = [m for m in matches if m is not None]
data = [json.loads(m.group(1)) for m in matches]
df = pd.DataFrame(data)

df = df.drop(index=df[(df['min_corr'] == 10000000) | (df['max_corr'] == -10000000)].index)
df.sort_values("azi_raw", inplace=True)

# Offset to the full .10s
df['min_corr'] /= 25600
df['max_corr'] /= 25600
df['min_ch_corr'] /= 25600
df['max_ch_corr'] /= 25600
df['azi_raw'] /=  25600

fig, (ax0, ax1, ax2) = plt.subplots(1, 3, num='Angle correction', sharey=True)
ax0: plt.Axes
ax1: plt.Axes
ax2: plt.Axes

ax0_r = None

for i, ax in enumerate([ax0, ax1, ax2]):
    df_i = df[df['field'] == i]
    ax.plot(df_i['azi_raw'], df_i['min_corr'], label='min_corr', marker='x', linestyle='None')
    ax.plot(df_i['azi_raw'], df_i['max_corr'], label='max_corr', marker='x', linestyle='None')
    ax.plot(df_i['azi_raw'], df_i['min_ch_corr'], label='min_ch_corr', marker='+', linestyle='None')
    ax.plot(df_i['azi_raw'], df_i['max_ch_corr'], label='max_ch_corr', marker='+', linestyle='None')

    if i == 0:
        ax0_r = ax_r = ax.twinx()
    else:
        ax_r = ax.twinx()
        ax_r.sharey(ax0_r) # type: ignore
    ax_r.plot(df_i['azi_raw'], df_i['min_out'], label='min_out', marker='1', linestyle='None', color='black')
    ax_r.plot(df_i['azi_raw'], df_i['max_out'], label='max_out', marker='2', linestyle='None', color='black')

    if i == 2:
        ax_r.set_ylabel('Out angle (deg)')
    ax_r.legend(loc='center right')
    ax.set_xlabel('Raw angle (deg)')
    if i == 0:
        ax.set_ylabel('Correction (deg)')

    raw_min = df_i['azi_raw'].min()
    raw_max = df_i['azi_raw'].max()
    out_min = df_i['min_out'].min()
    out_max = df_i['max_out'].max()
    corr_min = df_i['min_corr'].min()
    corr_max = df_i['max_corr'].max()
    out_diff_max = (df_i['max_out'] - df_i['min_out']).max()
    ax.set_title(f'Mirror {i}\nraw range {raw_min:.2f} - {raw_max:.2f} deg (={raw_max - raw_min:.2f} deg)\nout range {out_min:.2f} - {out_max:.2f} deg (={out_max - out_min:.2f} deg)\ncorrection {corr_min:.2f} - {corr_max:.2f} deg\nmax channel angle diff {out_diff_max:.2f} deg')
    ax.legend(loc='center left')

plt.show()
