# -*- coding: utf-8 -*-
"""
ip_simulator defines the classes and methods of different types of simulators.
"""

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import numpy as np
from scipy import integrate

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2018/8/5"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2018 Jialun Liu. All rights reserved."


# --------------------------------------------------------------------------- #
# MotionSimulator solver
# --------------------------------------------------------------------------- #
class MotionSolver(object):
    """
    MotionSimulator performs maneuvering simulations to predict ship dynamics.

    Coordinate system:
    The moving body-fixed coordinate system is clockwise,
    horizontal with x positive forward on midships (o),
    y positive starboard of ship center line,
    and z positive down from waterline (26th ITTC Quality Systems Group, 2011).

    Rudder angle:The rudder angle is defined positive counter clockwise.
    Other angles and angular speed are positive clockwise.
    Thus, a positive rudder angle (starboard) provides a positive turning moment,
    making the ship turn positive (starboard).

    Angles should be noted with affix of rad and deg to declare the current unit.
    For calculations, angles should be done in format of rad.
    For inputs and outputs, angles should be done in format of deg.
    """

    def __init__(self, t_start=0.0, t_step=0.1, t_end=1.0):
        self.t_start = t_start
        self.t_step = t_step
        self.t_end = t_end
        self.n_step = round((t_end - t_start) / t_step)

        # time_step is rounded for the ode solver.
        self.t_accuracy = len(str(self.t_step - int(self.t_step))) - 2

    def __str__(self):
        return "\n".join(["%s: %s" % item for item in self.__dict__.items()])

    # --------------------------------------------------------------------------- #
    # Integrate derivatives
    # --------------------------------------------------------------------------- #
    def solve(self, field, vessel):
        # Initialise outputs
        vessel.set_start_state()

        time_hist = [self.t_start]
        state_hist = [vessel.start_state]

        # Config ODE solver
        ode_solver = integrate.ode(vessel.get_ode_dot)
        ode_solver.set_integrator("vode")  # ODE45 solver
        # ode_solver.set_integrator("vode", method="bdf", order=15)  # ODE15s solver
        ode_solver.set_f_params(field)
        ode_solver.set_initial_value(vessel.start_state, self.t_start)
        while ode_solver.successful() and round(ode_solver.t, self.t_accuracy) < self.t_end:
            vessel.set_start_state()
            ode_solver.integrate(ode_solver.t + self.t_step)

            time_hist.append(ode_solver.t)
            state_hist.append(ode_solver.y)

        return np.column_stack((np.array(time_hist), np.array(state_hist)))
