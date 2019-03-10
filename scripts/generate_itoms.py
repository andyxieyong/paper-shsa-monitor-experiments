#!/usr/bin/env python2.7
"""Saves some itoms to a pickle object file.

__date__ = 2019-03-09
__author__ = Denise Ratasich

"""

import argparse
import math
import numpy as np
import pickle

from model.itom import Itom


#
# config
#

signal2variable = {
    "/a/data": "x",
    "/b/data": "x",
    "/c/data": "x",
}
signals = signal2variable.keys()


#
# parse arguments
#

parser = argparse.ArgumentParser(description="""Save perfect/equal itoms to pickle object file.""")
parser.add_argument("-o", "--output", type=str, default="generated.obj",
                    help="""Output (pickle object file for run.py).""")
args = parser.parse_args()


#
# generate some perfect (equal and on time) itoms
#

t_reception = np.arange(0.5*1e9,15*1e9,1e9)  # in nsec similar to ROS timestamps
itoms = [(tr+n*0.15*1e9, Itom(s, 1.8+(-0.5)/10*tr/1e9, tr+n*0.15*1e9, signal2variable[s]))
         for tr in t_reception for n, s in enumerate(signals)]
print "[generate itoms] number of itoms: {}".format(len(itoms))


#
# dump data
#

data = {
    'manipulated': False,
    'itoms': itoms,
    'signals': signals,
    'faults': [],
}

with open(args.output, 'wb') as f:
    pickle.dump(data, f, protocol=-1)
