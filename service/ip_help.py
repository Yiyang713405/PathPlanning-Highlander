# -*- coding: utf-8 -*-
"""
Describe functions that are used throughout ICONS_Python and relevant projects.
"""

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import csv
import time

from numpy import array

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2018/9/18"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2018 Jialun Liu. All rights reserved."


# --------------------------------------------------------------------------- #
# Class Definition
# --------------------------------------------------------------------------- #
class IpTimer(object):
    """Record time elapsed during simulations.记录模拟过程中经过的时间"""

    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        # Time stamp of start
        self.timer_start = time.time()
        # Process timer starts
        self.process_timer_start = time.process_time()
        # Local timer starts
        # self.local_timer_start = time.asctime(time.localtime(self.timer_start))
        self.local_timer_start = time.ctime(self.timer_start)
        # print("Simulation starts at %s\n" % self.local_timer_start)

    def __exit__(self, type, value, traceback):
        # Time stamp of stop
        self.timer_stop = time.time()
        # Process timer stops
        self.process_timer_stop = time.process_time()
        # Local timer stops
        # self.local_timer_stop = time.asctime(time.localtime(self.timer_stop))
        self.local_timer_stop = time.ctime(self.timer_stop)
        # print("Simulation starts at %s\n" % self.local_timer_stop)

        print("Total Simulation Time: %.2f (s)" % (self.process_timer_stop - self.process_timer_start))


# --------------------------------------------------------------------------- #
# Function Definition
# --------------------------------------------------------------------------- #
def read_csv(file_name):
    """Read a csv file and return a numpy array."""

    if isinstance(file_name, str):
        with open(file_name, "rb") as f:
            data = [[float(item) for item in row] for row in csv.reader(f.read().splitlines()[1:])]
        return array(data)
    else:
        raise ValueError("Input file_name should be a string.")


def read_out(file_name):
    """Read a out file and return a numpy array."""
    if isinstance(file_name, str):
        with open(file_name, "r") as f:
            data = [[float(item) for item in line.split()] for line in f.read().splitlines()[6:]]
        return array(data)
    else:
        raise ValueError("Input file_name should be a strING.")

if __name__ == "__main__":
    IpTimer()
    print(IpTimer())
    print(IpTimer.__enter__(self).local_timer_start)
