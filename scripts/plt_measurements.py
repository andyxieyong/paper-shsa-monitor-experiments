#!/usr/bin/env python2.7
"""Plots the the input itoms at a specific point in time.

__date__ = 2019-03-06
__author__ = Denise Ratasich

"""

import argparse
import matplotlib.figure
import matplotlib.pyplot as plt
import numpy as np
import pickle

from model.itom import Itom, Itoms


# parse arguments
parser = argparse.ArgumentParser(description="""Plots distance measurements. Call bag_to_itoms.py with a bag file first.""")
parser.add_argument("picklefile", type=str,
                    help="""Data (pickle file) containing sequence of itoms.""")
parser.add_argument('-e', '--export', type=str,
                    help="Export figure to file.")
parser.add_argument("-t", "--timestamp", default=0.0, type=float,
                    help="""Time to visualize from start in seconds.""")
args = parser.parse_args()

# load data
with open(args.picklefile, 'rb') as f:
    data = pickle.load(f)


#
# prepare data frame
#

# field of view (scale measurements)
fov = {
    '/p2os/sonar/ranges': 180.0,
    '/scan/ranges': 270.0,
    '/tof_camera/frame/depth': 100.0,
}
invert = {
    '/p2os/sonar/ranges': 1.0,
    '/scan/ranges': -1.0,
    '/tof_camera/frame/depth': 1.0,
}

# sort by reception time
itoms = sorted(data['itoms'], key=lambda msg: msg[0])
t_start = itoms[0][0]
print t_start

# get nearest itoms to args.timestamp
time = {}
distance = {}  # outputs per substitution per monitor step
index = {}
for tr, itom in itoms:
    if tr >= t_start + args.timestamp*1e9:
        if 'dmin' in itom.name:
            # /emergency_stop/dmin of no interest
            continue
        time[itom.name] = itom.t
        v = itom.v
        if 'camera' in itom.name:
            # take 100 row
            # row width of depth image
            w = 320
            # take 100th row (about the height of lidar scan)
            h = 100
            v = [d for i, d in enumerate(itom.v) if i >= h*w and i < (h+1)*w]
        distance[itom.name] = list(v)
        num_ranges = len(v)
        index[itom.name] = np.linspace(fov[itom.name]/2*invert[itom.name],
                                       -fov[itom.name]/2*invert[itom.name], num=num_ranges)
        if len(distance.keys()) >= 3:
            break # all different itoms collected

print "time difference"
t_min = min(time.values())
for name, t in time.items():
    print float(t-t_min)/1e9, " - ", name

print "number of distance measurements:", len(distance)


#
# plot
#

font = {'family' : 'normal',
        'size'   : 16}
matplotlib.rc('font', **font)

params = matplotlib.figure.SubplotParams(left=0.05, right=0.95, bottom=0.12, top=0.95, hspace=0.1)

fig = plt.figure(figsize=(10,5), subplotpars=params)

for name, distance in distance.items():
    plt.plot(index[name], distance, label=name)
plt.xlabel("field of view to the front ($^\circ$)")
plt.xlim(-180, 180)
plt.xticks(range(-180, 180+45, 45))
plt.ylabel("distance (m)")
plt.ylim(0, 5)
plt.legend(loc='upper left', fontsize=16)

# save or show figure
if args.export:
    fig.savefig(args.export)
else:
    plt.show()
