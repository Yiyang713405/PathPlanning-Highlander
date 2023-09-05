# -*- coding: utf-8 -*-

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2020/10/8"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2020 Jialun Liu. All rights reserved."

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import numpy as np
from matplotlib import pyplot as plt

# --------------------------------------------------------------------------- #
# PyQt canvas
# --------------------------------------------------------------------------- #
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
# matplotlib.backends.backend_qt5agg用来连接Matplotlib和PyQt5。


class GroupBoxCanvas(FigureCanvasQTAgg):
    # GroupBoxCanvas [defines the attributes for piloting using Matplotlib through groupBox]

    def __init__(self, parent=None):
        self.fig = plt.figure()
        super(GroupBoxCanvas, self).__init__(self.fig)
        self.axes = self.fig.add_subplot(111)
        self.setParent(parent)


# --------------------------------------------------------------------------- #
# Maneuver
# --------------------------------------------------------------------------- #
class Maneuver(object):

    def __init__(self, maneuver_type, rps, delta_cmd_deg, psi_cmd_deg, action_time=0.0):
        self.maneuver_type = maneuver_type  # (-) Maneuver types
        self.rps = rps  # (-) RPS during the maneuver
        self.delta_cmd_deg = delta_cmd_deg  # (deg) Rudder angle requirement during the maneuver
        self.psi_cmd_deg = psi_cmd_deg  # (deg) Heading angle requirement during the maneuver
        self.action_time = action_time  # (s) Decide when the rudder angle is acted.

    def __str__(self):
        return "\n".join(["%s: %s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Maneuver simulator
# --------------------------------------------------------------------------- #
class SimulatorSolver(object):

    def __init__(self, t_start=0.0, t_step=1.0, t_end=200.0):
        self.t_start = t_start
        self.t_step = t_step
        self.t_end = t_end
        self.n_step = round((t_end - t_start) / t_step)

    def __str__(self):
        return "\n".join(["%s: %s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Path
# --------------------------------------------------------------------------- #
class Path(object):

    def __init__(self):
        self.count = 0  # Path path_point count
        self.coord = None  # Path coordinates in ECEF coordinate system
        self.num = 0  # Number of path points

    def set_num(self):
        self.num = self.coord.shape[0]

    def pick_point(self, path_slice):
        self.coord = self.coord[path_slice]
        self.set_num()

    def get_curr_point(self):
        return self.coord[self.count][:]

    def get_prev_point(self):
        if self.count == 0:
            print("WARNING! The current path_point is returned as it is the first path_point.")
            return self.coord[self.count][:]
        else:
            return self.coord[self.count - 1][:]

    def get_next_point(self):
        if self.count == self.num - 1:
            print("WARNING! The current path_point is returned as it is the last path_point.")
            return self.coord[self.count][:]
        else:
            return self.coord[self.count + 1][:]

    def add_point(self, path_point):
        if self.coord is None:
            self.coord = np.array(path_point, dtype=float)
        else:
            self.coord = np.row_stack((self.coord, np.array(path_point, dtype=float)))

        self.set_num()

    def reverse_coord(self):
        self.coord = np.flipud(self.coord)

    def reverse_count(self):
        self.count = self.num - self.count - 1

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])
