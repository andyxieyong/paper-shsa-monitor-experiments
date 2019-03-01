#!/usr/bin/env python2.7
"""Saves a bag file to pickle object file with a list of itoms.

__date__ = 2019-02-28
__author__ = Denise Ratasich

"""

import argparse
from interval import interval
import os.path
import pandas as pd
import rosbag
import roslib
import rospy
import rostopic
import subprocess
import yaml
import pickle

from model.itom import Itom, Itoms


#
# config
#

signal2topic = {
    "/p2os/sonar/ranges": "/p2os/sonar",
    "/scan/ranges": "/scan",
    "/emergency_stop/dmin/data": "/emergency_stop/dmin",
}
topic2signals = {
    "/p2os/sonar": ["/p2os/sonar/ranges"],
    "/scan": ["/scan/ranges"],
    "/emergency_stop/dmin": ["/emergency_stop/dmin/data"],
}
signal2variable = {
    "/p2os/sonar/ranges": "d_2d",
    "/scan/ranges": "d_2d",
    "/emergency_stop/dmin/data": "dmin",
}
signals = signal2topic.keys()
topics = set(signal2topic.values())


#
# parse arguments
#

parser = argparse.ArgumentParser(description="""Convert bag file to pickle object file.""")
parser.add_argument("bagfile", type=str,
                    help="""Recorded rosbag file including sensor logs
                    (input itoms).""")
parser.add_argument("-o", "--output", type=str,
                    help="""Output (pickle object file for run.py).""")
args = parser.parse_args()

if args.output is None:
    args.output = os.path.basename(args.bagfile) + ".obj"


#
# load data from bag file
#

bag = rosbag.Bag(args.bagfile)

# get info dict
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', args.bagfile], stdout=subprocess.PIPE).communicate()[0])
print info_dict
for t in topics:
    if t not in [info['topic'] for info in info_dict['topics']]:
        raise RuntimeError("topic {} not in bag file".format(t))


#
# filter messages for topics
#

msgs = []
for topic, msg, t in bag.read_messages():
    # t is reception time stamp!
    if topic in topics:
        stamp = msg.header.stamp.to_nsec()
        msgs.append((stamp, topic, msg))

# sort by time stamps
msgs = sorted(msgs, key=lambda msg: msg[0])

print "number of msgs: {}".format(len(msgs))


#
# convert messages to itoms
#

def get_topic_fields(signal):
    topic = signal2topic[signal]
    # split field(s) from topic
    fields = signal[len(topic)+1:]
    fields = fields.split("/") if len(fields) > 0 else []
    return fields

def convert_to_itom(signal, msg):
    fields = get_topic_fields(signal)
    ## COPIED from monitor_node.py
    v = msg
    for field in fields:
        v = getattr(v, field)
    itom = Itom(signal, v, msg.header.stamp.to_nsec(), signal2variable[signal])
    ## COPIED end
    return itom

# all itoms
itoms = []
for t, topic, msg in msgs:
    # a topic may contain several signals
    for s in topic2signals[topic]:
        itom = convert_to_itom(s, msg)
        itoms.append(itom)

print "number of itoms: {}".format(len(itoms))

# collect itoms for monitor steps
inputs = []
t_last = itoms[-1].t
itoms_per_step = Itoms()
# reverse order to take latest itom of an input signal
for n in range(len(itoms)-1, 0, -1):
    itom = itoms[n]
    # add itom if not yet used in step
    if itom.name not in itoms_per_step.keys():
        itoms_per_step[itom.name] = itom
    if len(itoms_per_step) < len(signals):
        continue
    # finish step
    inputs.insert(0, itoms_per_step)
    itoms_per_step = Itoms()

print "number of steps: {}".format(len(inputs))


#
# dump data
#

data = {
    'manipulated': False,
    'inputs': inputs,
}

with open(args.output, 'wb') as f:
    pickle.dump(data, f, protocol=-1)
