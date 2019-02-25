#!/usr/bin/env python2.7
"""Plot itoms.

__date__ = 2019-02-25
__author__ = Denise Ratasich

"""

import rosbag_pandas
import argparse
import matplotlib.pyplot as plt


# parse arguments
parser = argparse.ArgumentParser(description="""Plots results of demo.""")
parser.add_argument("bagfile", type=str,
                    help="""Recorded rosbag file containing monitor logs.""")
args = parser.parse_args()

# load data from bag file
df = rosbag_pandas.bag_to_dataframe(args.bagfile)

# plot
fig, ax = plt.subplots()
ax.plot(df.index, df['cmd_vel__linear_x'], marker='o', linestyle='')
plt.show()
