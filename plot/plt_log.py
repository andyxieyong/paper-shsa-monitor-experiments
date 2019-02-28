#!/usr/bin/env python2.7
"""Plots logs collected via log.launch.

__date__ = 2019-02-25
__author__ = Denise Ratasich

"""

import argparse
import matplotlib.colors as colors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import pandas as pd
import rosbag_pandas


# parse arguments
parser = argparse.ArgumentParser(description="""Plots logs of log.launch.""")
parser.add_argument("bagfile", type=str,
                    help="""Recorded rosbag file containing monitor logs.""")
args = parser.parse_args()

# load data from bag file
topics = ["/cmd_vel", "/emergency_stop/dmin", "/dmin_monitor/debug"]
df = rosbag_pandas.bag_to_dataframe(args.bagfile, include=topics)

# start with time 0 (timedelta to floats)
df.index = [(i - df.index[0]) for i in df.index]

print df.columns

# substitutions
substitutions = []
for c in df.columns:
    if '/dmin_monitor/debug/outputs' in c:
        split = c.split('/')
        substitutions.append(split[-2])
substitutions = sorted(set(substitutions))
print substitutions


#
# plot
#

fig, axes = plt.subplots(5, sharex=True)
axidx = 0

# plot cmd_vel
axes[axidx].plot(df.index, df['/cmd_vel/linear/x'],
                 label="/cmd_vel/linear/x", marker='.', linestyle='')
axes[axidx].plot(df.index, df['/cmd_vel/angular/z'],
                 label="/cmd_vel/angular/z", marker='.', linestyle='')
axes[axidx].set_ylabel("/cmd_vel")
axes[axidx].legend()

axidx = axidx + 1

# plot dmin
axes[axidx].plot(df.index, df['/emergency_stop/dmin/data'], label="/emergency_stop/dmin/data",
             marker='.', linestyle='')
axes[axidx].set_ylabel("/emergency_stop/dmin")
# plot dmin hysteresis
_, xmax = axes[axidx].get_xlim()
c = (1.0, 0, 0, 0.2)
bot, top = (0.49, 0.51)
hysteresis = patches.Rectangle((0,bot), xmax, top-bot,
                               edgecolor=c, facecolor=c)
axes[axidx].add_patch(hysteresis)

axidx = axidx + 1

# plot outputs
for o in substitutions:
    keybot = '/dmin_monitor/debug/outputs/' + o + '/bot'
    keytop = '/dmin_monitor/debug/outputs/' + o + '/top'
    yerr = df[keytop] - df[keybot]
    y = df[keybot] + yerr/2
    axes[axidx].errorbar(df.index, y, yerr=yerr, label="s{}".format(o),
                     marker='.', linestyle='')
axes[axidx].set_ylabel("output per substitution")
axes[axidx].legend()

axidx = axidx + 1

# plot error
num_s = 0
for c in df.columns:
    if '/dmin_monitor/debug/error' in c:
        split = c.split('/')
        axes[axidx].plot(df.index, df[c], label="s{}".format(split[-1]),
                     marker='.', linestyle='')
        num_s = num_s + 1
axes[axidx].set_ylabel("error sum per substitution")
axes[axidx].legend()

axidx = axidx + 1

# plot failed
axes[axidx].plot(df.index, df['/dmin_monitor/debug/failed'],
             marker='.', linestyle='')
axes[axidx].set_ylabel("index of failed substitution\n(-1 .. ok, none failed)")
axes[axidx].set_yticks(range(-1, num_s))
axes[axidx].set_ylim(-2, num_s)

axes[axidx].set_xlabel("time (s)")
plt.show()
