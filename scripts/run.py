#!/usr/bin/env python2.7
"""Run the pickle file generated by `align.py` or `bag_to_itoms.py`.

__date__ = 2019-02-28
__author__ = Denise Ratasich

"""

import argparse
import pickle

from model.itom import Itom, Itoms
from model.monitor import Monitor as SHSAMonitor


class Emulator(object):
    """Emulates monitor_node.py"""

    def __init__(self):
        self.__monitor = SHSAMonitor("../config/dmin.pl", 'dmin',
                                     librarypaths=["/python_ws/shsa-prolog/model"])
        self.__monitor.set_debug_callback(self.__debug_callback)

    def __validate(self, (outputs, values, error, failed)):
        assert self.__debug is not None
        # TODO check reproducability (output is the same like in the ROS run)
        assert self.__debug['failed'] == failed
        # reset debug callback
        self.__debug = None

    def __debug_callback(self, inputs, outputs, values, error, failed):
        self.__debug = {
            'inputs': inputs,
            'outputs': outputs,
            'values': values,
            'error': error,
            'failed': failed,
        }

    def __step(self, itoms):
        """Execute a monitor step."""
        failed = self.__monitor.monitor(itoms)

    def run(self, data):
        inputs = data['inputs']
        manipulated = True
        # in case align generated the data we have sth to compare to
        try:
            self.__manipulated = data['manipulated']
            outputs = data['outputs']
            assert len(inputs) == len(outputs)
        except Exception as e:
            pass
        for n in range(len(inputs)):
            # execute monitor
            self.__step(inputs[n])
            # check
            if not manipulated:
                self.__validate(outputs[n])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="""Run monitor with given sequence of itoms.""")
    parser.add_argument("picklefile", type=str,
                        help="""Data (pickle file) containing sequence of itoms.""")
    args = parser.parse_args()

    with open(args.picklefile, 'rb') as f:
        data = pickle.load(f)

    emulator = Emulator()
    emulator.run(data)