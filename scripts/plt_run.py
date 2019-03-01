#!/usr/bin/env python2.7
"""Plots the results of run.py.

__date__ = 2019-02-25
__author__ = Denise Ratasich

"""

import argparse
import matplotlib.colors as colors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import pandas as pd
import pickle

from model.itom import Itom, Itoms


# parse arguments
parser = argparse.ArgumentParser(description="""Plots output of run.py.""")
parser.add_argument("picklefile", type=str,
                    help="""Data (pickle file) containing sequence of itoms.""")
args = parser.parse_args()

# load data
with open(args.picklefile, 'rb') as f:
    data = pickle.load(f)


#
# prepare data frame
#

# substitutions
substitutions = data['substitutions']
for n, s in enumerate(substitutions):
    print "--- s{} ---\n{}".format(n, s)

def substitution_index(s):
    global substitutions
    return substitutions.index(s)


# debug outputs of monitor
# transpose
values = []  # output value per substitution
error = []  # error per substitution
failed = []  # index of substitution that failed
for _, v, e, f in data['outputs']:
    values.append(v)
    error.append(e)
    failed.append(f)
df_values = pd.DataFrame(values)
df_error = pd.DataFrame(error)
df_failed = pd.DataFrame(failed)


#
# plot
#

fig, axes = plt.subplots(3, sharex=True)
axidx = 0

# plot output values
for c in df_values.columns:
    axes[axidx].plot(df_values.index, df_values[c], label="s{}".format(c),
                     marker='.', linestyle='')
    # TODO errorbar
    # yerr = df[keytop] - df[keybot]
    # y = df[keybot] + yerr/2
    # axes[axidx].errorbar(df.index, y, yerr=yerr, label="s{}".format(o),
    #                  marker='.', linestyle='')
axes[axidx].set_ylabel("dmin\noutput per substitution")
axes[axidx].legend()

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

axes[axidx].set_xlabel("steps")
plt.show()
