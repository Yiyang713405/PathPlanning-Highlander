# -*- coding: utf-8 -*-
"""
Model-scale KVLCC2 (INSEAN)
Scale Factor: 45.714
Model length: 7.0 (m)
Reference:
% SIMMAN 2014: https://simman2014.dk/ship-data/moeri-container-ship/geometry-and-conditions-moeri-container-ship/#top
% SIMMAN 2020: http://simman2019.kr/contents/test_case_3.2.php
"""

import numpy as np

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import ip_class
import ip_save
from ip_field import OpenSea
from ip_vessel import Vessel, Hull, Propeller, Rudder
# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2018/7/15"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2018 Jialun Liu. All rights reserved."
# --------------------------------------------------------------------------- #
# Hull
# --------------------------------------------------------------------------- #
hull = Hull(1)

hull.LOA = 7.22  # (m)
hull.LPP = 7.0  # (m)
hull.LWL = 7.1204  # (m)
hull.B = 1.1688  # (m)
hull.BWL = 1.1688  # (m)
hull.D = 0.6563  # (m)
hull.T = 0.455  # (m)
hull.TFP = hull.T  # (m)
hull.TAP = hull.T  # (m)
hull.set_TMS()  # (m)
hull.set_TST()  # (m)
hull.set_TST2TMS()  # (1)
hull.set_LPP2BWL()  # (1)
hull.set_BWL2LPP()  # (1)
hull.set_BWL2TMS()  # (1)
hull.set_TMS2BWL()  # (1)
hull.set_LPP2TMS()  # (1)
hull.set_TMS2LPP()  # (1)

hull.CB = 0.8098  # (1)
hull.CM = 0.9980  # (1)
hull.CP = 0.8120  # (1)
hull.CPA = 0.7350  # (1)
hull.CWA = 0.8901  # (1)

hull.SBH = 13.0129  # (m^2)
hull.DISPV = 3.2724  # (m^3)

hull.ixx_pri = 0.4  # (1)
hull.iyy_pri = 0.24  # (1)
hull.izz_pri = 0.24  # (1)

hull.mx_pri_II = 0.02192  # (1)
hull.my_pri_II = 0.22290  # (1)
hull.Jzz_pri_II = 0.01060  # (1)

hull.GMT = 0.125  # (m)

hull.xB = 0.244  # (m)
hull.xF = 0.244  # (m)
hull.xG = 0.244  # (m)
hull.yG = 0.0  # (m)
hull.zG = 0.0  # (m)

hull.set_m(OpenSea.water.rho)
hull.set_pri_II_mass(OpenSea.water.rho)
hull.set_pri_II_inertia(OpenSea.water.rho)
hull.set_mx_my_Jzz_man()
hull.set_Izz_man()  # (Nm^2)

# Hydrodynamic coefficients
hull.R0_pri_II = -0.022  # (1)

hull.Xvv_pri_II = -0.040  # (1)
hull.Xvr_pri_II = 0.002  # (1)
hull.Xrr_pri_II = 0.011  # (1)
hull.Xvvvv_pri_II = 0.771  # (1)

hull.Yv_pri_II = -0.315  # (1)
hull.Yr_pri_II = 0.083  # (1)
hull.Yvvv_pri_II = -1.607  # (1)
hull.Yvvr_pri_II = 0.379  # (1)
hull.Yvrr_pri_II = -0.391  # (1)
hull.Yrrr_pri_II = 0.008  # (1)
hull.Nv_pri_II = -0.137  # (1)
hull.Nr_pri_II = -0.049  # (1)
hull.Nvvv_pri_II = -0.030  # (1)
hull.Nvvr_pri_II = -0.294  # (1)
hull.Nvrr_pri_II = 0.055  # (1)
hull.Nrrr_pri_II = -0.013  # (1)

# --------------------------------------------------------------------------- #
# Propeller
# --------------------------------------------------------------------------- #
propeller = Propeller(1)

propeller.type = "FPP"  # (1)

propeller.NPB = 4  # (1)
propeller.D = 0.216  # (m)
propeller.P = 0.1558  # (m)
propeller.PDR = 0.7212  # (m)
propeller.ADE = 0.448  # (m^2)

propeller.w0 = 0.4  # (1)
propeller.t0 = 0.220  # (1)

propeller.KTP = [-0.1385, -0.2753, 0.2931]  # (1)

propeller.x = -3.36  # (m)
propeller.y = 0.00  # (m)
propeller.z = 0.00  # (m)

propeller.rps_serv = 10.4  # (Hz)
propeller.rps_dot_serv = 10.0  # (1/s^2) rps changing rate

# --------------------------------------------------------------------------- #
# Rudder
# --------------------------------------------------------------------------- #
rudder = Rudder(1)
rudder.type = "spade"
rudder.profile = "NACA 0018"

rudder.SX = 0.0115  # (m^2)
rudder.SM = 0.0539  # (m^2)
rudder.ST = 0.0654  # (m^2)
rudder.B = 0.3455  # (m)
rudder.C = 0.1891  # (m)
rudder.AS = 1.827  # (1)

rudder.x = -3.5  # (m)
rudder.y = 0.00  # (m)
rudder.z = 0.00  # (m)

rudder.t = 0.387  # (1)

rudder.epsilon = 1.09
rudder.set_w0_Kijima1990(propeller)

rudder.ell_pri = -0.710
rudder.kappa = 0.5
rudder.aH = 0.312

rudder.xH_pri = -0.464
rudder.xH = rudder.xH_pri * hull.LPP

rudder.lambda_k = rudder.AS / (rudder.AS + 2.25)
rudder.D2B = propeller.D / rudder.B

# rudder.dCL2dSinAOA_inf_CFD_K1 = 6.0145
# rudder.dCL2dSinAOA_inf_CFD_K0 = 0.0084
# rudder.dCD2dSinAOA_inf_CFD_K1 = 0.0314
# rudder.dCD2dSinAOA_inf_CFD_K0 = 0.0091

rudder.delta_dot_serv_deg = 15.8  # (deg/s) Rudder angle changing rate

# --------------------------------------------------------------------------- #
# Vessel
# --------------------------------------------------------------------------- #
class Kvlcc2MarinL7(Vessel):

    def __init__(self):
        super(Kvlcc2MarinL7, self).__init__()

        self.vessel_name = "KVLCC2_MARIN_L7"
        self.vessel_doc = "Model-scale KVLCC2 tanker"
        self.vessel_type = "super_tanker"
        self.propulsion_type = "propeller_rudder"
        self.power_type = "electricity"

        self.n_hull = 1  # (1)
        self.n_propeller = 1  # (1)
        self.n_rudder = 1  # (1)

        self.SC = 45.714  # (1)
        # self.SC = 1  # (1)
        self.U_serv = 1.179  # (m/s)

        # self.loading = "ballast"
        self.loading = "full"

        self.hull = hull
        self.propeller = propeller
        self.rudder = rudder

        # Initial state values
        self.u = 1E-5  # (m/s)
        self.v = 0.0  # (m/s)
        self.w = 0.0  # (m/s)
        self.p_deg = 0.0  # (deg/s)
        self.q_deg = 0.0  # (deg/s)
        self.r_deg = 0.0  # (deg/s)
        self.n = 0.0  # (m)
        self.e = 0.0  # (m)
        self.d = 0.0  # (m)
        self.phi_deg = 0.0  # (deg)
        self.theta_deg = 0.0  # (deg)
        self.psi_deg = 0.0  # (deg)

        self.lon_deg = 0.0  # (deg)
        self.lat_deg = 0.0  # (deg)

        self.propeller.rps = 1E-5
        self.rudder.delta = 0.0

        self.propeller.rps_dot = self.propeller.rps_dot_serv  # (1/s^2)
        self.rudder.delta_dot_deg = self.rudder.delta_dot_serv_deg  # (deg/s)
        self.rudder.delta_max_deg = 35  # (deg)

        self.trans_p_deg2rad()
        self.trans_q_deg2rad()
        self.trans_r_deg2rad()
        self.trans_phi_deg2rad()
        self.trans_theta_deg2rad()
        self.trans_psi_deg2rad()
        self.trans_lon_deg2rad()
        self.trans_lat_deg2rad()

        self.rudder.trans_delta_rad2deg()
        self.rudder.trans_delta_dot_deg2rad()
        self.rudder.trans_delta_max_deg2rad()

        # Simulation parameters

        self.is_surge = True  # Is surge motion simulated?
        self.is_sway = True  # Is sway motion simulated?
        self.is_heave = False  # Is heave motion simulated?
        self.is_roll = False  # Is roll motion simulated?
        self.is_pitch = False  # Is pitch motion simulated?
        self.is_yaw = True  # Is yaw motion simulated?

        self.is_MID = True  # Integrate on the origin point, i.e. MID, or somewhere like COG
        self.is_COG = False  # Integrate on the origin point, i.e. MID, or somewhere like COG

        # Column index number of simulation results
        self.hist_idx_t = 0
        self.hist_idx_n = 1
        self.hist_idx_u = 2
        self.hist_idx_e = 3
        self.hist_idx_v = 4
        self.hist_idx_phi_deg = None
        self.hist_idx_p_deg = None
        self.hist_idx_psi_deg = 5
        self.hist_idx_r_deg = 6
        self.hist_idx_rps = 7
        self.hist_idx_delta_deg = 8

        # Column index number of validation data
        self.val_idx_t = 0
        self.val_idx_n = 1
        self.val_idx_u = 2
        self.val_idx_e = 3
        self.val_idx_v = 4
        self.val_idx_phi_deg = 5
        self.val_idx_p_deg = 6
        self.val_idx_psi_deg = 7
        self.val_idx_r_deg = 8
        self.val_idx_rps = 9
        self.val_idx_delta_deg = 10

    # --------------------------------------------------------------------------- #
    # Hull Forces
    # --------------------------------------------------------------------------- #
    def get_X_H_Yasukawa_2015(self):
        """
        Get X_H with Hydrodynamic coefficients expressed by v' and r'.

        Reference:
        Yasukawa, H. & Yoshimura, Y., 2015. Introduction of MMG Standard Method for
            self Maneuvering Predictions. Journal of Marine Science and Technology,
            20(1), pp.37–52.
        """
        X_H_pri_II = self.hull.R0_pri_II + \
                     self.hull.Xvv_pri_II * self.v_pri ** 2 + \
                     self.hull.Xvr_pri_II * self.v_pri * self.r_pri + \
                     self.hull.Xrr_pri_II * self.r_pri ** 2 + \
                     self.hull.Xvvvv_pri_II * self.v_pri ** 4

        X_H = X_H_pri_II * self.pri_II_force  # (N)
        return X_H

    def get_Y_H_Yasukawa_2015(self):
        """
        Get Y_H with Hydrodynamic coefficients expressed by v_pri and r_pri.

        Reference:
        Yasukawa, H. & Yoshimura, Y., 2015. Introduction of MMG Standard Method for
            self Maneuvering Predictions. Journal of Marine Science and Technology,
            20(1), pp.37–52.
        """

        Y_H_pri_II = self.hull.Yv_pri_II * self.v_pri + \
                     self.hull.Yr_pri_II * self.r_pri + \
                     self.hull.Yvvv_pri_II * self.v_pri ** 3 + \
                     self.hull.Yvvr_pri_II * self.v_pri ** 2 * self.r_pri + \
                     self.hull.Yvrr_pri_II * self.v_pri * self.r_pri ** 2 + \
                     self.hull.Yrrr_pri_II * self.r_pri ** 3

        Y_H = Y_H_pri_II * self.pri_II_force  # (N)

        return Y_H

    def get_N_H_Yasukawa_2015(self):
        """
        Get N_H with Hydrodynamic coefficients expressed by v_pri and r_pri.

        Reference:
        Yasukawa, H. & Yoshimura, Y., 2015. Introduction of MMG Standard Method for
            self Maneuvering Predictions. Journal of Marine Science and Technology,
            20(1), pp.37–52.
        """

        N_H_pri_II = self.hull.Nv_pri_II * self.v_pri + \
                     self.hull.Nr_pri_II * self.r_pri + \
                     self.hull.Nvvv_pri_II * self.v_pri ** 3 + \
                     self.hull.Nvvr_pri_II * self.v_pri ** 2 * self.r_pri + \
                     self.hull.Nvrr_pri_II * self.v_pri * self.r_pri ** 2 + \
                     self.hull.Nrrr_pri_II * self.r_pri ** 3

        N_H = N_H_pri_II * self.pri_II_moment  # (Nm)
        return N_H

    def get_hull_force(self):
        """
        Get X_H, Y_H, Z_H, K_H, N_H, N_H
        """
        X_H = self.get_X_H_Yasukawa_2015()
        Y_H = self.get_Y_H_Yasukawa_2015()
        Z_H = 0.0
        K_H = 0.0
        M_H = 0.0
        N_H = self.get_N_H_Yasukawa_2015()

        return X_H, Y_H, Z_H, K_H, M_H, N_H

    # --------------------------------------------------------------------------- #
    # Propeller Forces
    # --------------------------------------------------------------------------- #
    def get_prop_t(self):
        # In the MMG standard method(Yasukawa2015), tP is assumed to be constant
        # at given  propeller load. The steering effect on the propeller thrust
        # is excluded. The effect is taken into account at the rudder force.
        self.propeller.t = self.propeller.t0

    def get_prop_uv(self):
        """Get propeller thrust force X_P."""

        betaProp = self.beta - self.propeller.x / self.hull.LPP * self.r_pri
        # (rad) geometric inflow angle
        # The propeller force varies with the direction of the inflow

        C1 = 2.0
        if betaProp > 0:
            C2 = 1.6
        else:
            C2 = 1.1

        self.propeller.w = 1 - (1 - self.propeller.w0) * (1 + (1 - np.exp(-C1 * abs(betaProp))) * (C2 - 1))
        self.propeller.u = self.u * (1 - self.propeller.w)

    def get_prop_force(self, field):
        """
        Get X_P, Y_P, Z_P, K_P, M_P, N_P
        """
        self.get_prop_t()
        self.get_prop_uv()

        self.propeller.set_J()
        self.propeller.set_KT()
        self.propeller.set_T(field)

        X_P = (1 - self.propeller.t) * self.propeller.T  # (N)
        Y_P = 0.0
        Z_P = 0.0
        K_P = 0.0
        M_P = 0.0
        N_P = -self.propeller.y * X_P

        return X_P, Y_P, Z_P, K_P, M_P, N_P

    # --------------------------------------------------------------------------- #
    # Rudder Forces
    # --------------------------------------------------------------------------- #
    def get_rudd_w(self):
        self.rudder.w = 1 - self.rudder.epsilon * (1 - self.propeller.w)

    def get_rudd_uv_EFD(self):
        """Calculate the rudder inflow velocities by experimental coefficients."""
        betaRudd = self.beta - self.rudder.ell_pri * self.r_pri

        if betaRudd < 0:
            gamma_rudd = 0.395
        else:
            gamma_rudd = 0.640

        eta_rudd = self.propeller.D / self.rudder.B

        self.rudder.v = self.U * gamma_rudd * betaRudd
        self.get_rudd_w()

        self.rudder.u = (1 - self.rudder.w) * self.u * \
                        np.sqrt(eta_rudd * (1 + self.rudder.kappa * (np.sqrt(
                            1 + 8 * self.propeller.KT / (np.pi * self.propeller.J ** 2)) - 1)) ** 2 + 1 - eta_rudd)

        self.rudder.set_U()

    def get_rudd_FNFT(self, field):
        """
        Get the X_R, Y_R, and N_R by Yasukawa 2014.
        Yasukawa use rudder normal force to calculate X_R, Y_R, and N_R.
        """

        self.get_rudd_uv_EFD()

        delta_eff = self.rudder.delta - np.arctan(self.rudder.v / self.rudder.u)

        CN_R = 6.13 * self.rudder.lambda_k * np.sin(delta_eff)

        FR_N = 0.5 * field.water.rho * self.rudder.SM * CN_R * self.rudder.U ** 2  # (N)

        FR_T = 0.0

        return FR_N, FR_T

    def get_rudd_FXFY(self, field):
        FR_N, FR_T = self.get_rudd_FNFT(field)

        FR_X = FR_N * np.sin(self.rudder.delta) + FR_T * np.cos(self.rudder.delta)
        FR_Y = FR_N * np.cos(self.rudder.delta) - FR_T * np.sin(self.rudder.delta)

        return FR_X, FR_Y

    def get_rudd_force(self, field):
        """
        Get X_R, Y_R, Z_R, K_R, M_R, N_R
        """

        FR_X, FR_Y = self.get_rudd_FXFY(field)

        X_R = -(1 - self.rudder.t) * FR_X  # (N)
        Y_R = -(1 + self.rudder.aH) * FR_Y  # (N)
        Z_R = 0.0
        K_R = 0.0
        M_R = 0.0
        N_R = -(self.rudder.x + self.rudder.aH * self.rudder.xH) * FR_Y \
              + self.rudder.y * (1 - self.rudder.t) * FR_X  # (N)

        return X_R, Y_R, Z_R, K_R, M_R, N_R

    # --------------------------------------------------------------------------- #
    # Total Forces
    # --------------------------------------------------------------------------- #
    def get_total_force(self, field):
        """
        X_H, Y_H, N_H:
        Calculate by hydrodynamic coefficients expressed by v' and r'.
        Hydrodynamic coefficients are calculated by Kijima methods.
        X_P:
        Calculate by X_P =  (1 - tp) * TMS
        u_r, v_r:
        Calculate by empirical coefficients as described in Yasukawa 2014.
        X_R, Y_R, N_R:
        Calculate by Lift and Drag, which is calculated by Fujii's method.

        Reference:
        Yasukawa, H. & Yoshimura, Y., 2015. Introduction of MMG Standard Method for
            self Maneuvering Predictions. Journal of Marine Science and Technology,
            20(1), pp.37–52.

        Toxopeus, S., 2011. Practical application of viscous-flow calculations for
            the simulation of maneuvering ships.
        """
        X_H, Y_H, Z_H, K_H, M_H, N_H \
            = self.get_hull_force()
        X_P, Y_P, Z_P, K_P, M_P, N_P \
            = self.get_prop_force(field)
        X_R, Y_R, Z_R, K_R, M_R, N_R \
            = self.get_rudd_force(field)

        return X_H, Y_H, Z_H, K_H, M_H, N_H, \
               X_P, Y_P, Z_P, K_P, M_P, N_P, \
               X_R, Y_R, Z_R, K_R, M_R, N_R

    def get_ode_dot(self, _, ode_y, field):
        self.n = ode_y[0]
        self.u = ode_y[1]
        self.e = ode_y[2]
        self.v = ode_y[3]
        self.psi = ode_y[4]  # FIXME YF
        self.r = ode_y[5]
        self.propeller.rps = ode_y[6]
        self.rudder.delta = ode_y[7]

        self.trans_r_rad2deg()
        self.trans_psi_rad2deg()
        self.rudder.trans_delta_rad2deg()

        self.set_U()  # (m/s) KVLCC2 speed
        self.set_beta()  # (rad)  Drift angle
        self.set_pri_linear_speed()
        self.set_pri_angular_speed(self.hull)
        self.set_pri_II_force(field, self.hull)
        self.set_pri_II_moment(field, self.hull)

        self.u_pri = self.u / self.pri_linear_speed  # (1) Non-dimensional surge speed
        self.v_pri = self.v / self.pri_linear_speed  # (1) Non-dimensional sway speed
        self.r_pri = self.r / self.pri_angular_speed  # (1) Non-dimensional yaw rate

        # Solve forces
        X_H, Y_H, Z_H, K_H, M_H, N_H, \
        X_P, Y_P, Z_P, K_P, M_P, N_P, \
        X_R, Y_R, Z_R, K_R, M_R, N_R = self.get_total_force(field)

        X_tot = (X_H + X_P + X_R) * self.is_surge  # Total surge force on body along body x
        Y_tot = (Y_H + Y_P + Y_R) * self.is_sway  # Total sway force on body along body y
        # Z_tot = (Z_H + Z_P + Z_R) * self.is_heave  # Total heave force on body along body z
        # M_tot = (K_H + K_P + K_R) * self.is_roll  # Total roll moment on body around body x
        # K_tot = (M_H + M_P + M_R) * self.is_pitch  # Total pitch moment on body around body y
        N_tot = (N_H + N_P + N_R) * self.is_yaw  # Total yaw moment on body around body z

        # Update control orders
        self.propeller.set_rps_dot()
        self.rudder.set_delta_dot()

        # Config the mass matrix
        m22 = self.hull.m + self.hull.mx
        m23 = -(self.hull.m + self.hull.my) * self.r
        m25 = -self.hull.xG * self.hull.m * self.r
        m44 = self.hull.m + self.hull.my
        m45 = (self.hull.m + self.hull.mx) * self.u
        m46 = self.hull.xG * self.hull.m
        m64 = self.hull.xG * self.hull.m
        m65 = self.hull.xG * self.hull.m * self.u
        m66 = self.hull.Izz + self.hull.Jzz + self.hull.xG ** 2 * self.hull.m

        # Conduct the mass matrix
        mat_mass = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # u     = u
            [0.0, m22, m23, 0.0, m25, 0.0],  # u_dot = X_tot
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],  # v     = v
            [0.0, 0.0, 0.0, m44, m45, m46],  # v_dot = Y_tot
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],  # r     = r
            [0.0, 0.0, 0.0, m64, m65, m66],  # r_dot = N_tot
        ], dtype=float)

        # Conduct the force matrix
        mat_force = np.array([self.u, X_tot, self.v, Y_tot, self.r, N_tot], dtype=float)

        # Get state on midships in the body-fixed coordinate system
        self.u, self.u_dot, self.v, self.v_dot, self.r, self.r_dot = np.linalg.solve(mat_mass, mat_force)

        self.p = self.p * self.is_roll
        self.q = self.q * self.is_pitch
        self.w = self.w * self.is_heave

        if self.is_MID:
            # Form derivatives on the origin point MID for integration
            self.trans_MID_BODY2NED()  # Transform BODY velocity to NED velocity on COG
            ode_dot = np.array([
                self.n_dot, self.u_dot, self.e_dot, self.v_dot, self.r, self.r_dot,
                self.propeller.rps_dot, self.rudder.delta_dot
            ], dtype=float)

        else:
            # Form derivatives somewhere not MID like COG for integration
            self.trans_linear_MID2COG()  # Transform BODY velocity from MID to COG
            self.trans_COG_BODY2NED()  # Transform BODY velocity to NED velocity on COG

            ode_dot = np.array([
                self.nG_dot, self.uG_dot, self.eG_dot, self.vG_dot, self.r, self.r_dot,
                self.propeller.rps_dot, self.rudder.delta_dot
            ], dtype=float)
        return ode_dot

    def set_start_state(self):
        self.start_state = np.array([self.n, self.u, self.e, self.v, self.psi, self.r, self.propeller.rps, self.rudder.delta], dtype=float)
# --------------------------------------------------------------------------- #
# Main function as an example
# --------------------------------------------------------------------------- #

def main():
    import ip_transform
    import ip_solver
    import ip_plot
    import ip_help

    import matplotlib.pyplot as plt
    from ip_validate import provide_validation
    from ip_field import OpenSea

    # Initialization
    motion_solver = ip_solver.MotionSolver(t_start=0.0, t_step=0.1, t_end=200.0)

    ship_param = Kvlcc2MarinL7()
    field_param = OpenSea

    ship_param.u = ship_param.U_serv  # (m/s)
    ship_param.propeller.rps = ship_param.propeller.rps_serv  # (Hz)
    ship_param.rudder.delta_deg = 0.0  # (deg)
    ship_param.rudder.trans_delta_deg2rad()

    ship_param.propeller.rps_cmd = ship_param.propeller.rps_serv  # (Hz)
    ship_param.rudder.delta_cmd_deg = 35.0  # (deg)

    # Simulation
    print("\n" + "*" * 15 + " Start " + "*" * 15)
    with ip_help.IpTimer():
        sim_hist_ms_rad = motion_solver.solve(field_param, ship_param)
        sim_hist_ms_deg = ip_transform.hist_rad2deg(ship_param, sim_hist_ms_rad)
    print("*" * 16 + " End " + "*" * 16 + "\n")

    # Display and validation
    maneuver_param = ip_class.Maneuver("turning", ship_param.propeller.rps, 35, np.inf, 0.0)

    # Process
    sim_hist_fs_deg = ip_transform.hist_m2f(ship_param, sim_hist_ms_deg)
    # Plot
    val_hist_fs_deg = provide_validation(ship_param, maneuver_param, "full-scale")

    plt.close("all")
    fig, axes = plt.subplots(3, 3, dpi=None, figsize=(12, 9))
    ip_plot.plot_all_info(fig, axes, ship_param, sim_hist_fs_deg, val_hist_fs_deg)
    plt.show()

    val_hist_ms_deg = provide_validation(ship_param, maneuver_param, "model-scale")
    # plt.close("all")
    fig, axes = plt.subplots(3, 3, dpi=None, figsize=(12, 9))
    ip_plot.plot_all_info(fig, axes, ship_param, sim_hist_ms_deg, val_hist_ms_deg)
    plt.show()

    # Save
    ip_save.save_hist(sim_hist_ms_deg, ship_param.vessel_name + "_motion_ms_deg")


# --------------------------------------------------------------------------- #
# Main function for tests
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    main()
