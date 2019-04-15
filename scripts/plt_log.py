#!/usr/bin/env python2.7
"""Plots logs collected via log.launch.

__date__ = 2019-02-25
__author__ = Denise Ratasich

"""

import argparse
import matplotlib.colors as colors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import rosbag_pandas


# parse arguments
parser = argparse.ArgumentParser(description="""Plots logs of log.launch.""")
parser.add_argument("bagfile", type=str,
                    help="""Recorded rosbag file containing monitor logs.""")
parser.add_argument('-e', '--export', type=str,
                    help="Export figure to file.")
args = parser.parse_args()

# load data from bag file
topics = ["/p2os/cmd_vel", "/emergency_stop/dmin", "/dmin_monitor/debug"]
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

font = {'family' : 'normal',
        'size'   : 26}
matplotlib.rc('font', **font)

# 5 plots
#params = matplotlib.figure.SubplotParams(left=0.08, right=0.98, bottom=0.05, top=0.98, hspace=0.1)
# 2 plots
params = matplotlib.figure.SubplotParams(left=0.1, right=0.98, bottom=0.12, top=0.95, hspace=0.1)

fig, axes = plt.subplots(2, figsize=(15,8), sharex=True, subplotpars=params)
axidx = 0

# plot cmd_vel
dl = df['/p2os/cmd_vel/linear/x'].dropna()
da = df['/p2os/cmd_vel/angular/z'].dropna()
axes[axidx].plot(dl.index, dl,
                 label="/p2os/cmd_vel/linear/x", marker='.', linestyle='-')
axes[axidx].plot(da.index, da,
                 label="/p2os/cmd_vel/angular/z", marker='.', linestyle='-.')
axes[axidx].set_ylabel("$v_{cmd}$", fontsize=30)
axes[axidx].legend(loc='lower left', fontsize=26)

axidx = axidx + 1

# plot dmin
axes[axidx].plot(df.index, df['/emergency_stop/dmin/data'], label="/emergency_stop/dmin/data",
             marker='.', linestyle='')
axes[axidx].set_ylabel("$v_{dmin}$", fontsize=30)
axes[axidx].set_ylim(bottom=0)
axes[axidx].set_yticks([0,0.5,1,1.5])
# plot dmin hysteresis
_, xmax = axes[axidx].get_xlim()
c = (1.0, 0, 0, 0.2)
bot, top = (0.49, 0.51)
hysteresis = patches.Rectangle((0,bot), xmax, top-bot,
                               edgecolor=c, facecolor=c)
axes[axidx].add_patch(hysteresis)

# axidx = axidx + 1

# # plot outputs
# for o in substitutions:
#     keybot = '/dmin_monitor/debug/outputs/' + o + '/bot'
#     keytop = '/dmin_monitor/debug/outputs/' + o + '/top'
#     yerr = df[keytop] - df[keybot]
#     y = df[keybot] + yerr/2
#     axes[axidx].plot(df.index, y, label="s{}".format(o),
#                      marker='.', linestyle='')
# axes[axidx].set_ylabel("$\mathsf{v}_{dmin}$", fontsize=20)
# axes[axidx].set_ylim(bottom=0)
# axes[axidx].legend()

# axidx = axidx + 1

# # plot error
# num_s = 0
# for c in df.columns:
#     if '/dmin_monitor/debug/error' in c:
#         split = c.split('/')
#         axes[axidx].plot(df.index, df[c], label="s{}".format(split[-1]),
#                      marker='.', linestyle='')
#         num_s = num_s + 1
# axes[axidx].set_ylabel("error", fontsize=16)
# axes[axidx].set_ylim(bottom=0)
# axes[axidx].legend()

# axidx = axidx + 1

# # plot failed

# d = df['/dmin_monitor/debug/failed'].dropna()
# axes[axidx].plot(d.index, d,
#                  marker='.', linestyle='-')
# axes[axidx].set_ylabel("failed s", fontsize=16)
# axes[axidx].set_yticks(range(-1, num_s))
# axes[axidx].set_ylim(-2, num_s)

axes[axidx].set_xlabel("time (s)")
axes[axidx].set_xlim(right=23)

# save or show figure
if args.export:
    fig.savefig(args.export)
else:
    plt.show()
