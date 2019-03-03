#!/usr/bin/env python2.7
"""Plots the results of run.py.

__date__ = 2019-02-25
__author__ = Denise Ratasich

"""

import argparse
import matplotlib.colors as colors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.figure
import pandas as pd
import pickle

from model.itom import Itom, Itoms


# parse arguments
parser = argparse.ArgumentParser(description="""Plots output of run.py.""")
parser.add_argument("picklefile", type=str,
                    help="""Data (pickle file) containing sequence of itoms.""")
parser.add_argument('-e', '--export', type=str,
                    help="Export figure to file.")
args = parser.parse_args()

# load data
with open(args.picklefile, 'rb') as f:
    data = pickle.load(f)


#
# prepare data frame
#

# substitutions
signal2substitution_idx = {}
substitutions = data['substitutions']
for n, s in enumerate(substitutions):
    for v in s.vin:
        signal2substitution_idx[v.name] = n
    print "--- s{} ---\n{}".format(n, s)

def substitution_index(s):
    global substitutions
    return substitutions.index(s)


# debug outputs of monitor
# transpose
time = []  # timestamp of monitor executions
values = []  # output value per substitution
error = []  # error per substitution
failed = []  # index of substitution that failed
for t, _, v, e, f in data['outputs']:
    time.append(t)
    values.append(v)
    error.append(e)
    failed.append(f)
time_start = time[0]
time = [float(t - time_start)/1e9 for t in time]
df_values = pd.DataFrame(values, index=time)
df_error = pd.DataFrame(error, index=time)
df_failed = pd.DataFrame(failed, index=time)


#
# plot
#

params = matplotlib.figure.SubplotParams(left=0.1, right=0.98, bottom=0.08, top=0.98, hspace=0.1)
basecolors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
lightcolors = [colors.colorConverter.to_rgba(c, alpha=0.3) for c in basecolors]

fig, axes = plt.subplots(4, figsize=(10,8), sharex=True, subplotpars=params)
axidx = 0

# plot output values
for i, c in enumerate(df_values.columns):
    try:
        axes[axidx].plot(df_values.index, df_values[c], label="s{}".format(c),
                         color=basecolors[i], marker='.', linestyle='')
    except ValueError as e:
        # df_values contains intervals -> plot midpoint and error
        lo = [v[0][0] for v in df_values[c]]
        hi = [v[0][1] for v in df_values[c]]
        mid = [v.midpoint[0][0] for v in df_values[c]]
        axes[axidx].fill_between(df_values.index, y1=lo, y2=hi, step='pre',
                                 facecolor=lightcolors[i])
        axes[axidx].plot(df_values.index, mid, label="s{}".format(c),
                         color=basecolors[i], marker='.', linestyle='')
axes[axidx].set_ylabel("dmin\noutput per substitution")
axes[axidx].legend()

axidx = axidx + 1

# plot injected faults
print data['faults']
for t0, t1, signal, desc in data['faults']:
    t0 = (float(t0) - time_start)/1e9
    t1 = (float(t1) - time_start)/1e9
    y = signal2substitution_idx[signal]
    axes[axidx].plot([t0, t1], [y, y], marker='s', linestyle='-', linewidth=2)
    axes[axidx].text(t0, y-0.5, "{}\n{}".format(signal, desc))
axes[axidx].set_ylabel("faults injected")
axes[axidx].set_yticks(range(0, len(substitutions)))
axes[axidx].set_ylim(-1.5, len(substitutions)-0.5)

axidx = axidx + 1

# plot error
for c in df_error.columns:
    axes[axidx].plot(df_error.index, df_error[c], label="s{}".format(c),
                     marker='.', linestyle='')
axes[axidx].set_ylabel("error sum per substitution")
axes[axidx].legend()

axidx = axidx + 1

# plot failed
axes[axidx].plot(df_failed.index, df_failed[0],
                 marker='.', linestyle='-')
axes[axidx].set_ylabel("index of failed substitution\n(-1 .. ok, none failed)")
axes[axidx].set_yticks(range(-1, len(substitutions)))
axes[axidx].set_ylim(-1.5, len(substitutions)-0.5)

axes[axidx].set_xlabel("time (s)")

# save or show figure
if args.export:
    fig.savefig(args.export)
else:
    plt.show()
