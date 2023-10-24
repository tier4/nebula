import argparse
import json
import re
import pandas as pd
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser(description='Analyze angle timing')
parser.add_argument('log_file')

args = parser.parse_args()

with open(args.log_file, 'r') as f:
    lines = f.readlines()

matches = [re.search(r'\[ANGDBG\]\s+(\{.*?\})', line) for line in lines]
matches = [m for m in matches if m is not None]
data = [json.loads(m.group(1)) for m in matches]
df = pd.DataFrame(data)

t0 = df['t_ns'].iloc[0]

# Throw away first 10 sec (stabilization)
df = df.drop(index=df[(df['azi_out'] < 0) | (df['t_ns'] < t0 + 10e9)].index)

# Offset to the full .10s
df['tos_offset'] = df['t_ns'] % int(1e8)

fig, axs = plt.subplots(2, 2, num='Angle timing')
ax0 = axs[0, 0]
ax1 = axs[0, 1]
ax2 = axs[1, 0]
ax3 = axs[1, 1]

ax0: plt.Axes
ax1: plt.Axes

# Scatter plot of encoder angle to corrected angle
ax0.scatter(df['azi_raw'], df['azi_out'])
ax0.axhline(df['azi_out'].min())
ax0.axhline(df['azi_out'].max())
ax0.set_title('Raw angle vs. out angle')
ax0.set_xlabel('Raw angle (deg)')
ax0.set_ylabel('Out angle (deg)')

# Scatter plot of corrected angle to ToS offset
scan_phase = df['scan_phase'].iloc[0]
ax1.scatter(df['azi_raw'], df['tos_offset'])
ax1.axvline(x=scan_phase)
ax1.axhline(y=0)  # top of second offset
ax1.set_title(f'Raw angle vs. ToS offset {scan_phase=}')
ax1.set_xlabel('Raw angle (deg)')
ax1.set_ylabel('ToS offset (ns)')

scan_phase = df['scan_phase'].iloc[0]
ax3.scatter(df['azi_out'], df['tos_offset'])
ax3.axvline(x=scan_phase)
ax3.axhline(y=0)  # top of second offset
ax3.set_title(f'Out angle vs. ToS offset {scan_phase=}')
ax3.set_xlabel('Out angle (deg)')
ax3.set_ylabel('ToS offset (ns)')

ax2.plot(df['t_ns'], df['azi_raw'])
plt.show()
