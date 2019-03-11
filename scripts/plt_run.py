#!/usr/bin/env python2.7
"""Plots the results of run.py.

__date__ = 2019-02-25
__author__ = Denise Ratasich

"""

import argparse
from interval import interval
import matplotlib.colors as colors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.figure
import numpy as np
import pandas as pd
import pickle
import yaml

from model.itom import Itom, Itoms


# parse arguments
parser = argparse.ArgumentParser(description="""Plots output of run.py.""")
parser.add_argument("picklefile", type=str,
                    help="""Data (pickle file) containing sequence of itoms.""")
parser.add_argument("-u", "--uncertainty", type=str,
                    help="""Configuration file of the uncertainty model for errorbars.""")
parser.add_argument('-p', '--plot-uncertainty', action='store_true',
                    help="""Plot uncertainty as error bars.""")
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
print "----------"

# offset to original value given uncertainty interval
offset = {n: (0, 0) for n in range(len(substitutions))}
terr = {n: [0, 0] for n in range(len(substitutions))}
verr = {n: [0, 0] for n in range(len(substitutions))}

# uncertainty per substitution
# (interval has only boundaries, original point is lost != midpoint)
if args.uncertainty:
    with open(args.uncertainty, 'r') as ymlfile:
        uncertainty = yaml.load(ymlfile)
        for n, s in enumerate(substitutions):
            assert len(s.vin) == 1
            i = list(s.vin)[0].name
            toff = -float(uncertainty[i]['terr_low'])/1e9 # to seconds
            voff = -uncertainty[i]['verr_low']
            offset[n] = (toff, voff)
            terr[n] = [float(uncertainty[i]['terr_low'])/1e9,
                       float(uncertainty[i]['terr_high'])/1e9]
            verr[n] = [uncertainty[i]['verr_low'],
                       uncertainty[i]['verr_high']]


# debug outputs of monitor (monitor timeline)
df = pd.DataFrame(data['outputs'])

t0 = df['tm'].iloc[0]
print "start:", t0

t_rel = lambda t_ns: float(t_ns - t0)/1e9

df['tm'] = df['tm'].apply(t_rel)
df.set_index('tm', inplace=True)

df_error = pd.DataFrame(df['error'].values.tolist(), index=df.index)
df_outputs = pd.DataFrame(df['outputs'].values.tolist(), index=df.index)

def oitoms(df):
    df_oitoms = []
    for tidx, oinfo in enumerate(df['outputs'].values.tolist()):
        for o in oinfo:
            # save related timestamp of monitor call
            o['tm'] = df.index[tidx]
            # prepare data frame for v and t intervals
            toff, voff = offset[o['sidx']]
            o['v_orig'] = o['v'][0][0] + voff
            o['t_orig'] = t_rel(o['t'][0][0] + toff)
            df_oitoms.append(o)
    return pd.DataFrame(df_oitoms)

df_oitoms = oitoms(df)


#
# plot
#

font = {'family' : 'normal',
        'size'   : 22}
matplotlib.rc('font', **font)

params = matplotlib.figure.SubplotParams(left=0.06, right=0.99, bottom=0.08, top=0.98, hspace=0.1)
basecolors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
markers = ['o', 's', 'd', 'v']
lightcolors = [colors.colorConverter.to_rgba(c, alpha=0.3) for c in basecolors]

fig, axes = plt.subplots(3, figsize=(16,12), sharex=True, subplotpars=params)
axidx = 0

# plot output values
for sidx in [0, 1, 2, 3]:
    is_sidx = (df_oitoms['sidx'] == sidx)
    df2plt = df_oitoms[is_sidx]
    n = len(df2plt['v_orig'])
    xerr = [[-terr[sidx][0]]*n, [terr[sidx][1]]*n]
    yerr = [[-verr[sidx][0]]*n, [verr[sidx][1]]*n]
    axes[axidx].errorbar(df2plt['t_orig'], df2plt['v_orig'],
                         xerr=xerr, yerr=yerr,
                         label="s{}".format(sidx), color=basecolors[sidx],
                         marker=markers[sidx], linestyle='')
axes[axidx].set_ylabel("output")
axes[axidx].set_ylim(-0.2,2.5)
axes[axidx].legend(loc='lower left')

axidx = axidx + 1

if data['manipulated']:
    # plot injected faults
    for ts, te, signal, desc in data['faults']:
        y = signal2substitution_idx[signal]
        axes[axidx].plot([t_rel(ts), t_rel(te)], [y, y],
                         color=basecolors[y],
                         marker='s', linestyle='-', linewidth=2)
        axes[axidx].text(t_rel(ts), y-0.8, "{}".format(desc))
    axes[axidx].set_ylabel("faults injected")
    axes[axidx].set_yticks(range(0, len(substitutions)))
    axes[axidx].set_ylim(-1.5, len(substitutions)-0.5)
else:
    # plot error
    for c in df_error.columns:
        axes[axidx].plot(df_error.index, df_error[c], label="s{}".format(c),
                         marker=markers[int(c)], linestyle='')
    axes[axidx].set_ylabel("error")
    axes[axidx].set_ylim(bottom=0)
    axes[axidx].legend(loc='lower left')

axidx = axidx + 1

# plot failed idx
axes[axidx].plot(df.index, df['failed_idx'],
                 marker='.', linestyle='-')
axes[axidx].set_ylabel("failed")
axes[axidx].set_yticks(range(-1, len(substitutions)))
axes[axidx].set_ylim(-1.5, len(substitutions)-0.5)

axes[axidx].set_xlim(0,23)
axes[axidx].set_xticks(range(0,23))
axes[axidx].set_xlabel("time (s)")

# save or show figure
if args.export:
    fig.savefig(args.export)
else:
    plt.show()
