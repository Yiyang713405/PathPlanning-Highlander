# -*- coding: utf-8 -*-

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2018/7/15"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2018 Jialun Liu. All rights reserved."

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import numpy as np


# --------------------------------------------------------------------------- #
# Hull
# --------------------------------------------------------------------------- #
class Hull(object):

    def __init__(self, id_hull, loading_condition="full"):
        # --------------------------------------------------------------------------- #
        # Hull geometric parameters
        # --------------------------------------------------------------------------- #
        self.id = id_hull  # (1) Unique id of the hull
        self.loading = loading_condition  # (1) Loading condition of the hull, i.e. full, ballast, etc.
        self.LOA = None  # (m) Length overall of a ship
        self.LPP = None  # (m) Length between perpendiculars
        self.LWL = None  # (m) Length at waterline
        self.B = None  # (m) Moulded beam of a ship
        self.BWL = None  # (m) Beam at waterline
        self.D = None  # (m) Moulded depth of a ship
        self.T = None  # (m) Design draft of a ship
        self.TFP = None  # (m) Draft at forward perpendicular
        self.TAP = None  # (m) Draft at aft perpendicular
        self.TMS = None  # (m) Draft at midships
        self.TST = None  # (m) Static trim

        self.TST2TMS = None  # (1) Ratio of static trim to mean draft
        self.LPP2BWL = None  # (1) Ratio of length between perpendiculars and beam at waterline
        self.BWL2LPP = None  # (1) Ratio of beam at waterline to length between perpendiculars
        self.BWL2TMS = None  # (1) Ratio of beam at waterline to draft
        self.TMS2BWL = None  # (1) Ratio of draft to beam at waterline
        self.LPP2TMS = None  # (1) Ratio of length between perpendiculars and mean draft
        self.TMS2LPP = None  # (1) Ratio of mean draft to length between perpendiculars

        self.CB = None  # (1) Block coefficient
        self.CM = None  # (1) Midships section coefficient
        self.CP = None  # (1) Longitudinal prismatic coefficient
        self.CPF = None  # (1) Prismatic coefficient, after body
        self.CPA = None  # (1) Prismatic coefficient, fore body
        self.CW = None  # (1) Full ship water plane area coefficient
        self.CWF = None  # (1) Water plane area coefficient, forward
        self.CWA = None  # (1) Water plane area coefficient, aft
        self.CX = None  # (1) Full ship central lateral plane coefficient in Holtrop methods
        self.CXA = None  # (1) Full ship central lateral plane coefficient for the aft ship Holtrop methods

        self.SBH = None  # (m^2) Bare hull wet surface area, underway

        self.DISPV = None  # (m^3) Displacement volume
        self.DISPF = None  # (N) Displacement force

        self.m = None  # (kg) KVLCC2 mass
        self.W = None  # (N) Weight of the body, i.e. W = m g
        self.m_pri_II = None  # (-) Non-dimensional ship mass
        self.mx = None  # (kg) Added mass in x direction
        self.mx_pri_II = None  # (-) Non-dimensional added mass in x direction
        self.my = None  # (kg) Added mass in y direction
        self.my_pri_II = None  # (-) Non-dimensional added mass in y direction

        self.ixx_pri = None  # (1) Radius of gyration for roll non-dimensionlized by B in air
        self.iyy_pri = None  # (1) Radius of gyration for pitch non-dimensionlized by LPP in air
        self.izz_pri = None  # (1) Radius of gyration for yaw non-dimensionlized by LPP in air

        self.Ixx = None  # (kg m^2) Roll moment of inertia around axis x
        self.Iyy = None  # (kg m^2) Pitch moment of inertia around axis y
        self.Izz = None  # (kg m^2) Yaw moment of inertia around axis z

        self.Jxx = None  # (kg m^2) Added roll moment of inertia around axis x
        self.Jyy = None  # (kg m^2) Added pitch moment of inertia around axis y
        self.Jzz = None  # (kg m^2) Added yaw moment of inertia around axis z

        self.Jxx_pri_II = None  # (kg m^2) Non-dimensional added roll moment of inertia around axis x
        self.Jyy_pri_II = None  # (kg m^2) Non-dimensional added pitch moment of inertia around axis y
        self.Jzz_pri_II = None  # (kg m^2) Non-dimensional added yaw moment of inertia around axis z

        self.xB = None  # (m) Longitudinal coordinate of the center of buoyancy, i.e. CB
        self.yB = None  # (m) Transversal coordinate of the center of buoyancy, i.e. CB
        self.zB = None  # (m) Normal coordinate of the center of buoyancy, i.e. CB

        self.LCB = None  # (%) Position of the centre of buoyancy forward of 0.5L as a percentage of ship length.

        self.xG = None  # (m) Longitudinal coordinate of the center of gravity, i.e. CG
        self.yG = None  # (m) Transversal coordinate of the center of gravity, i.e. CG
        self.zG = None  # (m) Normal coordinate of the center of gravity, i.e. CG

        self.xF = None  # (m) Longitudinal coordinate of the center of flotation as resultant of weight and buoyancy
        self.yF = None  # (m) Transversal coordinate of the center of flotation as resultant of weight and buoyancy
        self.zF = None  # (m) Normal coordinate of the center of flotation as resultant of weight and buoyancy

        self.xM = None  # (m) Longitudinal coordinate of the center of metacenter, i.e. CM
        self.yM = None  # (m) Transversal coordinate of the center of metacenter, i.e. CM
        self.zM = None  # (m) Normal coordinate of the center of metacenter, i.e. CM

        self.GM = None  # (m) Normal metacentric height the distance from CB to CM, positive to upward
        self.GMT = None  # (m) Transversal metacentric height

        # Parameters for Holtrop methods
        self.Holtrop_At = None  # (m^2) Area of transom
        # Transverse sectional area of the bulb where the still-water surface intersects the stern
        self.Holtrop_Abt = 0  # (m^2)
        self.Holtrop_hB = None  # (1)
        self.Holtrop_C_stern = None  # (1)

        # --------------------------------------------------------------------------- #
        # Hull hydrodynamic parameters
        # --------------------------------------------------------------------------- #
        self.R0 = None  # (N) Resistance in straight moving
        self.R0_pri_II = None  # (1) Resistance coefficient in straight moving

        self.RT = None  # (N) Total resistance, which is equivalent to R0 in straight moving
        self.CRT = None  # (N) Total resistance coefficient

        # Non-dimensional factors using prime system I
        self.pri_I_length = None  # (1)
        self.pri_I_area = None  # (1)
        self.pri_I_mass = None  # (1)
        self.pri_I_inertia = None  # (1)
        # Non-dimensional factors using prime system II
        self.bis_length = None  # (1)
        self.bis_area = None  # (1)
        self.bis_mass = None  # (1)
        self.bis_inertia = None  # (1)
        # Non-dimensional factors using bis system
        self.pri_II_length = None  # (1)
        self.pri_II_area = None  # (1)
        self.pri_II_mass = None  # (1)
        self.pri_II_inertia = None  # (1)

    def set_TMS(self):
        self.TMS = (self.TFP + self.TAP) / 2  # (m) Draft at midships, TMS = (TFP + TAP)/2

    def set_TST(self):
        self.TST = self.TAP - self.TFP  # (m) Static trim, TST = TAP - TFP

    def set_TST2TMS(self):
        self.TST2TMS = self.TST / self.TMS  # (1) Ratio of trim to draft

    def set_LPP2BWL(self):
        self.LPP2BWL = self.LPP / self.BWL  # (1) Length to beam ratio

    def set_BWL2LPP(self):
        self.BWL2LPP = self.BWL / self.LPP  # (1) Beam to length ratio

    def set_BWL2TMS(self):
        self.BWL2TMS = self.BWL / self.TMS  # (1) Beam to draft ratio

    def set_TMS2BWL(self):
        self.TMS2BWL = self.TMS / self.BWL  # (1) Draft to beam ratio

    def set_LPP2TMS(self):
        self.LPP2TMS = self.LPP / self.TMS  # (1) Length to draft ratio

    def set_TMS2LPP(self):
        self.TMS2LPP = self.TMS / self.LPP  # (1) Length to draft ratio

    def set_m(self, rho):
        self.m = self.DISPV * rho  # (kg) Ship mass

    def set_m_pri_II(self):
        self.m_pri_II = self.m / self.pri_II_mass  # (1) Non-dimensional ship mass

    def set_mx_pri_II(self):
        self.mx_pri_II = self.mx / self.pri_II_mass  # (1) Non-dimensional ship mass

    def set_my_pri_II(self):
        self.my_pri_II = self.my / self.pri_II_mass  # (1) Non-dimensional ship mass

    def set_mx_my_Jzz_man(self):
        """Get mx, my and Jzz by manually put Non-dimensional mx, my and Jzz
        coefficients."""

        self.mx = self.mx_pri_II * self.pri_II_mass  # (kg)
        self.my = self.my_pri_II * self.pri_II_mass  # (kg)
        self.Jzz = self.Jzz_pri_II * self.pri_II_inertia  # (kg m^2)

    def set_mx_my_Jzz_Zhou1983(self):
        """Based on reference of Zhou1983."""
        self.mx = (0.398 + 11.97 * self.CB * (1 + 3.73 * self.TMS2BWL) - 2.89 * self.CB * self.LPP2BWL *
                   (1 + 1.13 * self.TMS2BWL) + 0.175 * self.CB * self.LPP2BWL ** 2 *
                   (1 + 0.54 * self.TMS2BWL) - 1.107 * self.LPP2BWL * self.TMS2BWL) * self.m / 100  # (kg)

        self.my = (0.882 - 0.54 * self.CB * (1 - 1.6 * self.TMS2BWL) - 0.156 *
                   (1 - 0.673 * self.CB) * self.LPP2BWL + 0.826 * self.TMS2BWL * self.LPP2BWL *
                   (1 - 0.678 * self.TMS2BWL) - 0.638 * self.CB * self.TMS2BWL * self.LPP2BWL *
                   (1 - 0.669 * self.TMS2BWL)) * self.m  # (kg)

        jz = (33 - 76.85 * self.CB * (1 - 0.784 * self.CB) + 3.43 * self.LPP2BWL *
              (1 - 0.63 * self.CB)) * self.LPP / 100  # (m)

        self.Jzz = self.m * jz ** 2  # (kg m^2)

    def set_mx_my_Jzz_Clarke1983(self):
        """Based on reference of Clarke1983."""
        self.mx = 0.06 * self.m  # mx = (0.03~0.06)m
        self.my = self.pri_II_mass * np.pi * self.TMS2LPP * (
                1 + 0.16 * self.CB * self.BWL2TMS - 5.1 * self.BWL2LPP ** 2)
        self.Jzz = self.pri_II_inertia * np.pi * self.TMS2LPP * (
                1 / 12 + 0.017 * self.CB * self.BWL2TMS - 0.33 * self.BWL2LPP)

    def set_mx_my_Jzz_Sadakane2001(self):
        """Based on reference of Sadakane2001"""

        self.mx = (0.2425 * self.TMS2BWL * self.CB * self.BWL2LPP + 0.2737 * self.CB * self.BWL2LPP +
                   0.0120 * self.TMS2BWL - 0.0001) * self.pri_II_mass

        self.my = (0.0090 * self.TMS2BWL * self.CB * self.LPP2BWL - 0.2395 * self.CB * self.LPP2BWL +
                   3.8295 * self.TMS2BWL + 0.5892) * self.pri_II_mass

        self.Jzz = (0.5470 * self.TMS2BWL * self.CB * self.LPP2BWL - 1.1750 * self.CB * self.LPP2BWL +
                    6.9125 * self.TMS2BWL + 2.9880) * self.pri_II_inertia

    def set_Izz_Motora1959(self):
        izz = 0.2536 * self.LPP
        self.Izz = self.m * izz ** 2  # (kg m^2)

    def set_Izz_CCS2003(self):
        self.Izz = (1 + self.CB ** 4.5) * (self.LPP ** 2 + self.BWL ** 2) * self.m / 24  # (kg m^2)

    def set_Izz_man(self):
        izz = self.izz_pri * self.LPP  # (m)
        self.Izz = self.m * izz ** 2  # (kg m^2)

    def set_DISPF(self):
        self.DISPF = self.DISPV * 1E3  # (N)

    def set_LCB(self):
        self.LCB = 100 * self.xB / self.LPP  # (%)

    """
    Using SNAME1950 for prime system I:
    SNAME1950-Nomenclature for treating the motion of a submerged body through a fluid

    Using Yasukawa2015 for prime system II
    Yasukawa2015-Introduction of {MMG} standard method for ship maneuvering predictions

    Using Norrbin1970 for bis system:
    Norrbin1970-Theory and observation on the use of a mathematical model for ship maneuvering in deep and confined waters

    For the bias system, the body density ratio mu = m/(rho*V), where m is the mass unit and V is the hull contour displacement.
    mu varies with different types of vehicles as follows:
    mu < 1: Under vehicles (ROVs, AUVs and submarines)
    mu = 1: Floating ships/rigs and neutrally buoyant underwater vehicles
    mu > 1: Heavy torpedoes (typically mu = 1.3-1.5)
    """

    def set_pri_I_length(self):
        # Transform dimensional length to non-dimensional length using prime I system
        self.pri_I_length = self.LPP

    def set_pri_II_length(self):
        # Transform dimensional length to non-dimensional length using prime II system
        self.pri_II_length = self.LPP

    def set_bis_length(self):
        # Transform dimensional length to non-dimensional length using bis system
        self.bis_length = self.LPP

    def set_pri_I_area(self):
        # Transform dimensional area to non-dimensional area using prime I system
        self.pri_I_area = self.LPP ** 2

    def set_pri_II_area(self):
        # Transform dimensional area to non-dimensional area using prime II system
        self.pri_II_area = self.LPP * self.TMS

    def set_bis_area(self, mu=1.0):
        # Transform dimensional area to non-dimensional area using bis system
        self.bis_area = mu * 2 * self.DISPV / self.LPP

    def set_pri_I_mass(self, rho):
        # Transform dimensional mass to non-dimensional mass using prime I system
        self.pri_I_mass = 0.5 * rho * self.LPP ** 3

    def set_pri_II_mass(self, rho):
        # Transform dimensional mass to non-dimensional mass using prime II system
        self.pri_II_mass = 0.5 * rho * self.LPP ** 2 * self.TMS

    def set_bis_mass(self, rho, mu=1.0):
        # Transform dimensional mass to non-dimensional mass using bis system
        self.bis_mass = mu * rho * self.DISPV

    def set_pri_I_inertia(self, rho):
        # Transform dimensional inertia moment to non-dimensional inertia moment using prime I system
        self.pri_I_inertia = 0.5 * rho * self.LPP ** 5

    def set_pri_II_inertia(self, rho):
        # Transform dimensional inertia moment to non-dimensional inertia moment using prime II system
        self.pri_II_inertia = 0.5 * rho * self.LPP ** 4 * self.TMS

    def set_bis_inertia(self, rho, mu=1.0):
        # Transform dimensional inertia moment to non-dimensional inertia moment using bis system
        self.bis_inertia = mu * rho * self.DISPV * self.LPP ** 2

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Propeller
# --------------------------------------------------------------------------- #
class Propeller(object):

    def __init__(self, id_propeller):
        self.id = id_propeller  # (1) Unique id of the propeller
        self.type = None  # (-) FPP (fixed pitch propeller), CPP (controllable pitch propeller)
        # --------------------------------------------------------------------------- #
        # Propeller geometric parameters
        # --------------------------------------------------------------------------- #
        self.NPB = None  # (1) Number of propeller blades
        self.D = None  # (m) Propeller diameter
        self.P = None  # (m) Propeller pitch in general
        self.PDR = None  # (1) Pitch ratio
        self.ADE = None  # (1) Expanded blade area ratio

        self.w = None  # (1) Propeller Taylor wake fraction during maneuvering
        self.w0 = None  # (1) Propeller Taylor wake fraction in Straight moving
        self.t = None  # (1) Propeller thrust deduction faction/during maneuvering
        self.t0 = None  # (1) Propeller thrust deduction faction in Straight moving

        self.x = None  # (m) Longitudinal propeller position from midships
        self.y = None  # (m) Transversal propeller position from midships
        self.z = None  # (m) Normal propeller position

        # --------------------------------------------------------------------------- #
        # Propeller hydrodynamic parameters
        # --------------------------------------------------------------------------- #
        self.KTP = None  # (1) Thrust coefficient polynomials
        self.KT = None  # (1) Thrust coefficient

        self.KQP = None  # (1) Torque coefficient polynomials
        self.KQ = None  # (1) Torque coefficient

        self.u = None  # (m/s)
        self.v = None  # (m/s)

        self.J = None  # (1) Propeller advance ratio
        self.T = None  # (N) Propeller thrust

        self.beta = None  # (rad)

        # --------------------------------------------------------------------------- #
        # Propeller operation parameters
        # --------------------------------------------------------------------------- #
        self.rps_serv = None  # (Hz) Service propeller revolution per second
        self.rps = None  # (Hz) Propeller revolution per second
        self.rpm = None  # () Propeller revolution per minute

        self.rps_dot_serv = None  # (1/s^2) Service changing rate of rps
        self.rps_dot = None  # (1/s^2) Current changing rate of rps

        self.rps_cmd = None  # (Hz) RPS command

    def set_J(self):
        # Get propeller advance ratio
        self.J = self.u / (self.rps * self.D)

    def set_KT(self):
        # Get propeller thrust coefficient
        self.KT = np.polyval(self.KTP, self.J)

    def set_T(self, field):
        # Get propeller thrust force
        self.T = field.water.rho * self.rps ** 2 * self.D ** 4 * self.KT  # (N)

    def trans_rps2rpm(self):
        # Get RPM from RPS
        self.rpm = self.rps * 60.0

    def trans_rpm2rps(self):
        # Get RPS from RPM
        self.rps = self.rpm / 60.0

    def set_rps_dot(self):
        """
        Define the performance of the propulsion engine.
        rps increases or decreases with service rps changing rate.

        Returns:
            float: Propeller revolution rate (1/s^2)
        """
        if round(self.rps, 2) < round(self.rps_cmd, 2):
            self.rps_dot = self.rps_dot_serv
        elif round(self.rps, 2) > round(self.rps_cmd, 2):
            self.rps_dot = -self.rps_dot_serv
        else:
            self.rps_dot = 0.0

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# SideThruster
# --------------------------------------------------------------------------- #
class SideThruster(Propeller):

    def __init__(self):
        super().__init__(1)

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Azipod
# --------------------------------------------------------------------------- #
class Azipod(Propeller):
    def __init__(self):
        super().__init__(1)
        self.SM = None  # (m^2) Lateral area of the pod

        self.delta_dot = None  # (rad/s) Current rudder angle changing rate in radium
        self.delta_dot_deg = None  # (deg/s) Current rudder angle changing rate in degree
        self.delta_dot_serv_deg = None  # (deg/s) Service rudder angle changing rate in degree

        self.delta = None  # (rad) Current rudder angle in radium
        self.delta_hydro = None  # (rad) Hydrodynamic angle of attack
        self.delta_eff = None  # (rad) Effective angle of attack delta_eff = delta - delta_hydro

        self.delta_deg = None  # (deg) Current rudder angle in degree

        self.delta_cmd = None  # (deg) Command rudder angle in radium
        self.delta_cmd_deg = None  # (deg) Command rudder angle in degree

        self.delta_max = None  # (deg) Maximum rudder angle in radius
        self.delta_max_deg = None  # (deg) Maximum rudder angle in degree

    # --------------------------------------------------------------------------- #
    # Define transformers for rudder parameters
    # --------------------------------------------------------------------------- #
    def trans_delta_deg2rad(self):
        self.delta = np.deg2rad(self.delta_deg)

    def trans_delta_rad2deg(self):
        self.delta_deg = np.rad2deg(self.delta)

    def trans_delta_max_deg2rad(self):
        self.delta_max = np.deg2rad(self.delta_max_deg)

    def trans_delta_max_rad2deg(self):
        self.delta_max_deg = np.rad2deg(self.delta_max)

    def trans_delta_deg_deg2rad(self):
        self.delta_cmd = np.deg2rad(self.delta_cmd_deg)

    def trans_delta_cmd_rad2deg(self):
        self.delta_cmd_deg = np.rad2deg(self.delta_cmd)

    def trans_delta_cmd_deg2rad(self):
        self.delta_cmd = np.deg2rad(self.delta_cmd_deg)

    def trans_delta_dot_deg2rad(self):
        self.delta_dot = np.deg2rad(self.delta_dot_deg)

    def trans_delta_dot_rad2deg(self):
        self.delta_dot_deg = np.rad2deg(self.delta_dot)

    def set_delta_dot(self):
        """
        steering_engine defines the performance of the steering engine.
        delta increases or decreases with service rudder turning rate.

        Returns:
            float: Rudder turning rate delta_dot (rad).
        """

        if round(self.delta_deg, 2) < round(self.delta_cmd_deg, 2):
            self.delta_dot_deg = self.delta_dot_serv_deg
        elif round(self.delta_deg, 2) > round(self.delta_cmd_deg, 2):
            self.delta_dot_deg = -self.delta_dot_serv_deg
        else:
            self.delta_dot_deg = 0.0

        self.delta_dot = np.deg2rad(self.delta_dot_deg)

    def __str__(self):
        return '\n'.join(['%s:\t%s' % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Azimuth thruster
# --------------------------------------------------------------------------- #
class AzimuthThruster(Propeller):
    def __init__(self):
        super().__init__(1)

    def __str__(self):
        return '\n'.join(['%s:\t%s' % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Rudder
# --------------------------------------------------------------------------- #
class Rudder(object):

    def __init__(self, id_rudder):
        self.id = id_rudder  # (1) Unique id of the rudder
        self.type = None  # (-) Spade, flap, etc.
        self.profile = None  # (-) NACA, HSV, etc.
        # --------------------------------------------------------------------------- #
        # Rudder geometric parameters
        # --------------------------------------------------------------------------- #
        self.SX = None  # (m^2) Lateral area of the fixed part of rudder
        self.SM = None  # (m^2) Lateral area of movable part of rudder
        self.ST = None  # (m^2) Lateral rudder area
        self.B = None  # (m) Rudder span
        self.C = None  # (m) Mean chord length
        self.AS = None  # (1) Rudder aspect ratio

        self.x = None  # (m) Longitudinal rudder position from midships
        self.y = None  # (m) Transversal rudder position from midline
        self.z = None  # (m) Normal rudder position
        # --------------------------------------------------------------------------- #
        # Rudder hydrodynamic parameters
        # --------------------------------------------------------------------------- #
        self.w = None  # (1) Rudder Taylor wake fraction during maneuvering
        self.w0 = None  # (1) Rudder Taylor wake fraction in Straight moving
        self.t = None  # (1) Rudder thrust deduction faction/during maneuvering
        self.t0 = None  # (1) Rudder thrust deduction faction in Straight moving

        self.CL = None  # (1) Lift coefficient
        self.CD = None  # (1) Drag coefficient
        self.CN = None  # (1) Normal force coefficient
        self.CT = None  # (1) Tangential force coefficient

        self.FL = None  # (1) Lift force
        self.FD = None  # (1) Drag force
        self.FN = None  # (1) Normal force
        self.FT = None  # (1) Tangential force
        self.FX = None  # (1) Longitudinal force
        self.FY = None  # (1) Lateral force

        self.Re = None  # (1) Reynolds number

        # --------------------------------------------------------------------------- #
        # Rudder operation parameters
        # --------------------------------------------------------------------------- #
        self.u = None  # (m/s)
        self.v = None  # (m/s)
        self.U = None  # (m/s)

        self.beta = None  # (rad) Rudder drift angle
        self.gamma = None  # (1) Rudder flow straightening factor
        self.kappa = None  # (1) An experimental constant for expressing u_R
        self.eta = None  # (1) Ratio of propeller diameter to rudder span
        self.epsilon = None  # (1) Ratio of wake fraction at propeller and rudder positions epsilon = (1-wR)/(1-wP)
        self.f_alpha = None  # (1) A factor of Fujii's formula
        self.ell_pri = None  # (1) Effective longitudinal coordinate of rudder position in formula of beta_R
        self.xH_pri = None  # (1) Longitudinal coordinate of acting point of the additional lateral force
        self.xH = None  # (m)
        self.aH = None  # (1) Rudder force increase factor

        self.delta_dot = None  # (rad/s) Current rudder angle changing rate in radium
        self.delta_dot_deg = None  # (deg/s) Current rudder angle changing rate in degree
        self.delta_dot_serv_deg = None  # (deg/s) Service rudder angle changing rate in degree

        self.delta = None  # (rad) Current rudder angle in radium
        self.delta_hydro = None  # (rad) Hydrodynamic angle of attack
        self.delta_eff = None  # (rad) Effective angle of attack delta_eff = delta - delta_hydro

        self.delta_deg = None  # (deg) Current rudder angle in degree

        self.delta_cmd = None  # (deg) Command rudder angle in radium
        self.delta_cmd_deg = None  # (deg) Command rudder angle in degree

        self.delta_max = None  # (deg) Maximum rudder angle in radius
        self.delta_max_deg = None  # (deg) Maximum rudder angle in degree

    # --------------------------------------------------------------------------- #
    # Define transformers for rudder parameters
    # --------------------------------------------------------------------------- #
    def trans_delta_deg2rad(self):
        self.delta = np.deg2rad(self.delta_deg)

    def trans_delta_rad2deg(self):
        self.delta_deg = np.rad2deg(self.delta)

    def trans_delta_max_deg2rad(self):
        self.delta_max = np.deg2rad(self.delta_max_deg)

    def trans_delta_max_rad2deg(self):
        self.delta_max_deg = np.rad2deg(self.delta_max)

    def trans_delta_deg_deg2rad(self):
        self.delta_cmd = np.deg2rad(self.delta_cmd_deg)

    def trans_delta_cmd_rad2deg(self):
        self.delta_cmd_deg = np.rad2deg(self.delta_cmd)

    def trans_delta_cmd_deg2rad(self):
        self.delta_cmd = np.deg2rad(self.delta_cmd_deg)

    def trans_delta_dot_deg2rad(self):
        self.delta_dot = np.deg2rad(self.delta_dot_deg)

    def trans_delta_dot_rad2deg(self):
        self.delta_dot_deg = np.rad2deg(self.delta_dot)

    # --------------------------------------------------------------------------- #
    # Set rudder parameters
    # --------------------------------------------------------------------------- #
    def set_eta(self, propeller):
        self.eta = propeller.D / self.B  # (1)

    def set_ST(self):
        # Get total rudder area from fixed and movable area
        self.ST = self.SM + self.SX

    def set_U(self):
        self.U = np.sqrt(self.u ** 2 + self.v ** 2)

    def set_Re(self, field):
        self.Re = field.water.rho * self.U * self.C / field.mu

    def set_f_alpha_Fujii(self):
        # Get fAlpha by Fujii's formula.
        self.f_alpha = (6.13 * self.AS) / (2.25 + self.AS)

    def set_CL_CD_Fujii(self):
        # Get Cl by Fujii's formula.#
        self.CL = (6.13 * self.AS) / (2.25 + self.AS)
        self.CD = ((6.13 * self.AS) / (2.25 + self.AS)) ** 2 / np.pi * self.AS

    def set_kappa_Yoshimura2003(self, hull):
        # Yoshimura2003 Flow straightening coefficient
        self.kappa = 0.55 - 0.8 * hull.CB * hull.BWL2LPP

    def set_epsilon_Kijima1990(self, hull):
        # Kijima1990 epsilon_w0w0P = (1 - w0)/(1 - w0)
        self.epsilon = -156.2 * (hull.CB * hull.BWL2LPP) ** 2 + 41.6 * (hull.CB * hull.BWL2LPP) - 1.76

    def set_w0_Kijima1990(self, propeller):
        # Kijima1990 epsilon_w0w0P = (1 - w0)/(1 - w0)
        self.w0 = 1.0 - self.epsilon * (1.0 - propeller.w0)

    def set_w_Furukawa2008(self, vessel):
        zeta = 1.0
        self.w = zeta * self.w0 * np.exp(-4.0 * (vessel.beta + vessel.r_pri / 2) ** 2)

    def set_gamma_Yoshimura2012(self, hull):
        # Yoshimura2012: Static gamma for fishing ships and merchant ships
        self.gamma = 2.06 * hull.CB * hull.BWL2LPP + 0.14

    def set_ell_pri_Yoshimura2003(self, hull):
        # Yoshimura2003
        self.ell_pri = 1.7 * hull.CB * hull.BWL2LPP - 1.2

    def set_beta_Yasukawa2015(self, vessel):
        self.beta = vessel.beta - self.ell_pri * vessel.r_pri

    def set_uv_Yoshimura1989(self, vessel, propeller):
        # Yoshimura1978 - Modeling of Manoeuvring Behaviour of Ships with a Propeller Idling, Boosting and Reversing
        self.v = vessel.U * self.gamma * self.beta
        self.u = (1 - self.w) * (vessel.u - self.y * vessel.r) * np.sqrt(self.eta * (1 + self.kappa * (
                np.sqrt(1 + 8 * propeller.KT / (np.pi * propeller.J ** 2)) - 1)) ** 2 + 1 - self.eta)

    def set_delta_eff_Yasukawa2015(self):
        self.delta_hydro = np.arctan(self.v / self.u)
        self.delta_eff = self.delta - self.delta_hydro

    def set_t_Matsumoto1980(self, hull):
        # Matsumoto1980, applied by Kijima1990
        self.t = -0.28 * hull.CB + 0.45

    def set_aH_Quadvlieg2013b(self, hull):
        # For inland vessels
        self.aH = 0.627 * hull.CB - 0.153

    def set_xH_pri_Kijima1990(self, hull):
        self.xH_pri = np.polyval([9.5727, -8.0704, -0.0618], hull.CB)

    def self_xH_pri_Lee1998(self, hull):
        self.xH_pri = np.polyval([-148.44, 58.18, -6.054], hull.BWL2LPP)

    def set_xH(self, hull):
        self.xH = self.xH_pri * hull.LPP

    def set_FL_FD(self, field):
        self.FL = 0.5 * field.water.rho * self.SM * self.CL * self.U ** 2
        self.FD = 0.5 * field.water.rho * self.SM * self.CD * self.U ** 2

    def trans_FLFD_FXFY(self):
        self.FX = self.FL * np.sin(self.delta_hydro) + self.FD * np.cos(self.delta_hydro)
        self.FY = self.FL * np.cos(self.delta_hydro) - self.FD * np.sin(self.delta_hydro)

    def set_delta_dot(self):
        """
        steering_engine defines the performance of the steering engine.
        delta increases or decreases with service rudder turning rate.

        Returns:
            float: Rudder turning rate delta_dot (rad).
        """

        if round(self.delta_deg, 2) < round(self.delta_cmd_deg, 2):
            self.delta_dot_deg = self.delta_dot_serv_deg
        elif round(self.delta_deg, 2) > round(self.delta_cmd_deg, 2):
            self.delta_dot_deg = -self.delta_dot_serv_deg
        else:
            self.delta_dot_deg = 0.0

        self.delta_dot = np.deg2rad(self.delta_dot_deg)

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Fin
# --------------------------------------------------------------------------- #
class Fin(Rudder):

    def __init__(self):
        super().__init__(1)

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Servo
# --------------------------------------------------------------------------- #
class Servo(object):
    def __init__(self, power):
        self.power = None  # (w)

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# DieselEngine
# --------------------------------------------------------------------------- #
class DieselEngine(object):

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# ElectricEngine
# --------------------------------------------------------------------------- #
class ElectricEngine(object):

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Vessel
# --------------------------------------------------------------------------- #
class Vessel(object):
    """A class of ships with a single screw and a single rudder."""

    def __init__(self):
        # --------------------------------------------------------------------------- #
        # General arrangements
        # --------------------------------------------------------------------------- #
        self.vessel_name = None  # (-) Vessel name including ship length like KVLCC2_L7.
        self.vessel_doc = None  # (-) Documentation of the hull
        self.vessel_type = None  # (-) Vessel type like tanker, container, etc.
        self.propulsion_type = None  # (-) Propulsion type like propeller-rudder systems.
        self.power_type = None  # (-) Power type like electricity, diesel, etc.

        self.n_hull = 1  # (1) Number of hulls
        self.n_propeller = 1  # (1) Number of propellers
        self.n_rudder = 1  # (1) Number of rudders
        self.n_bow_thruster = 0  # (1) Number of bow thrusters
        self.n_stern_thruster = 0  # (1) Number of stern thrusters
        self.n_thruster = 0  # (1) Number of thrusters

        self.SC = 1.0  # (1) Scale ratio of the full-scale size to the model-size
        self.U_serv = None  # (m/s) Service speed of the ship

        self.loading = None  # (1) Loading condition like ballast and full

        self.hull = None
        self.propeller = None
        self.rudder = None
        self.bow_thruster = None
        self.stern_thruster = None
        self.azimuth_thruster = None

        self.hull_list = []  # List of hulls
        self.propeller_list = []  # List of propellers
        self.rudder_list = []  # List of rudders

        self.SAPP = 0.0  # (m^2) Appendage wet surface area
        self.AWS = 0.0  # (m^2) Area of wet surface/Wet surface area, underway

        # --------------------------------------------------------------------------- #
        # State parameters in Body-fixed coordinate system
        # --------------------------------------------------------------------------- #
        # self.x = None
        # self.y = None
        # self.z = None
        self.u = None
        self.v = None
        self.w = None
        self.p = None
        self.p_deg = None
        self.q = None
        self.q_deg = None
        self.r = None
        self.r_deg = None
        self.u_dot = None
        self.v_dot = None
        self.w_dot = None
        self.p_dot = None
        self.q_dot = None
        self.r_dot = None
        self.p_dot_deg = None
        self.q_dot_deg = None
        self.r_dot_deg = None
        self.uG = None
        self.vG = None
        self.wG = None
        self.uG_dot = None
        self.vG_dot = None
        self.wG_dot = None

        self.U = None
        self.u_pri = None
        self.v_pri = None
        self.w_pri = None
        self.p_pri = None
        self.q_pri = None
        self.r_pri = None

        self.beta = None
        self.beta_deg = None

        self.pri_time = None
        self.pri_linear_speed = None
        self.pri_angular_speed = None
        self.pri_linear_acc = None
        self.pri_angular_acc = None
        self.pri_I_force = None
        self.pri_II_force = None
        self.pri_I_moment = None
        self.pri_II_moment = None

        self.bis_time = None
        self.bis_linear_speed = None
        self.bis_angular_speed = None
        self.bis_linear_acc = None
        self.bis_angular_acc = None
        self.bis_force = None
        self.bis_moment = None

        self.Re = None  # (1) Reynolds number
        self.Fr = None  # (1) Froude number

        # --------------------------------------------------------------------------- #
        # State parameters in north-east-down coordinate system (NED)
        # --------------------------------------------------------------------------- #
        self.n = None
        self.e = None
        self.d = None
        self.nG = None
        self.eG = None
        self.dG = None
        self.n_dot = None
        self.e_dot = None
        self.d_dot = None
        self.nG_dot = None
        self.eG_dot = None
        self.dG_dot = None
        self.phi = None
        self.phi_dot = None
        self.phi_deg = None
        self.theta = None
        self.theta_dot = None
        self.theta_deg = None
        self.psi = None  # (rad) Heading angle in radium
        self.psi_dot = None  # (rad/s) Heading angle in radium
        self.psi_deg = None  # (deg) Heading angle in degree

        self.chi = None  # (rad) Course angle in radium
        self.chi_deg = None  # (deg) Course angle in degree

        # --------------------------------------------------------------------------- #
        # State parameters in earth-fixed coordinate system (ECEF)
        # --------------------------------------------------------------------------- #
        self.xE = None  # (m) Coordinate in ECEF coordinate system
        self.xE_dot = None  # (m)
        self.yE = None  # (m) Coordinate in ECEF coordinate system
        self.yE_dot = None  # (m)
        self.zE = None  # (m) Coordinate in ECEF coordinate system
        self.zE_dot = None  # (m)

        self.lon = None  # (rad) Longitude in radium
        self.lat = None  # (rad) Latitude in radium
        self.hgt = None  # (m) Height

        self.lon_deg = None  # (deg) Longitude in degree
        self.lat_deg = None  # (deg) Latitude in degree

        # --------------------------------------------------------------------------- #
        # State parameters in plotting coordinate system (MATLAB and Matplotlib)
        # --------------------------------------------------------------------------- #
        self.xP = None
        self.yP = None
        self.zP = None

        # --------------------------------------------------------------------------- #
        # State parameters in vision coordinate system (OpenCV, Unity and PyQt)
        # --------------------------------------------------------------------------- #
        self.xV = None
        self.yV = None
        self.zV = None

        self.mat_linear_b2n = None
        self.mat_linear_n2b = None
        self.mat_angular_b2n = None
        self.mat_angular_n2b = None
        self.mat_b2n = None
        self.mat_n2b = None
        self.mat_linear_n2e = None
        self.mat_n2b = None

        # --------------------------------------------------------------------------- #
        # State commands
        # --------------------------------------------------------------------------- #
        self.psi_cmd = None
        self.psi_cmd_deg = None

        # --------------------------------------------------------------------------- #
        # Simulation states related
        # --------------------------------------------------------------------------- #
        self.is_surge = None  # Is surge motion simulated?
        self.is_sway = None  # Is sway motion simulated?
        self.is_heave = None  # Is heave motion simulated?
        self.is_roll = None  # Is roll motion simulated?
        self.is_pitch = None  # Is pitch motion simulated?
        self.is_yaw = None  # Is yaw motion simulated?

        self.is_MID = None  # Integrate on the origin point, i.e. MID, or somewhere like COG
        self.is_COG = None  # Integrate on the origin point, i.e. MID, or somewhere like COG

        self.start_state = None  # Initial state for simulations

        # Column index number of simulation results
        self.hist_idx_t = None
        self.hist_idx_n = None
        self.hist_idx_u = None
        self.hist_idx_e = None
        self.hist_idx_v = None
        self.hist_idx_phi_deg = None
        self.hist_idx_p_deg = None
        self.hist_idx_psi_deg = None
        self.hist_idx_r_deg = None
        self.hist_idx_rps = None
        self.hist_idx_delta_deg = None

        # Column index number of validation data
        self.val_idx_t = None
        self.val_idx_n = None
        self.val_idx_u = None
        self.val_idx_e = None
        self.val_idx_v = None
        self.val_idx_phi_deg = None
        self.val_idx_p_deg = None
        self.val_idx_psi_deg = None
        self.val_idx_r_deg = None
        self.val_idx_rps = None
        self.val_idx_delta_deg = None

    def update_ship_state(self, sim_hist_temp):
        if self.hist_idx_n is not None:
            self.n = sim_hist_temp[-1, self.hist_idx_n]
        if self.hist_idx_u is not None:
            self.u = sim_hist_temp[-1, self.hist_idx_u]
        if self.hist_idx_e is not None:
            self.e = sim_hist_temp[-1, self.hist_idx_e]
        if self.hist_idx_v is not None:
            self.v = sim_hist_temp[-1, self.hist_idx_v]
        if self.hist_idx_phi_deg is not None:
            self.phi = sim_hist_temp[-1, self.hist_idx_phi_deg]
        if self.hist_idx_p_deg is not None:
            self.p = sim_hist_temp[-1, self.hist_idx_p_deg]
        if self.hist_idx_psi_deg is not None:
            self.psi = sim_hist_temp[-1, self.hist_idx_psi_deg]
        if self.hist_idx_r_deg is not None:
            self.r = sim_hist_temp[-1, self.hist_idx_r_deg]
        if self.hist_idx_rps is not None:
            self.propeller.rps = sim_hist_temp[-1, self.hist_idx_rps]
        if self.hist_idx_delta_deg is not None:
            self.rudder.delta = sim_hist_temp[-1, self.hist_idx_delta_deg]

    def get_SAPP(self):
        for rudder in self.rudder_list:
            self.SAPP += rudder.ST  # (m^2)

    def get_AWS(self):
        self.AWS = self.hull.SBH + self.SAPP  # (m^2)

    def trans_p_deg2rad(self):
        self.p = np.deg2rad(self.p_deg)

    def trans_p_rad2deg(self):
        self.p_deg = np.rad2deg(self.p)

    def trans_q_deg2rad(self):
        self.q = np.deg2rad(self.q_deg)

    def trans_q_rad2deg(self):
        self.q_deg = np.rad2deg(self.q)

    def trans_r_deg2rad(self):
        self.r = np.deg2rad(self.r_deg)

    def trans_r_rad2deg(self):
        self.r_deg = np.rad2deg(self.r)

    def trans_phi_deg2rad(self):
        self.phi = np.deg2rad(self.phi_deg)

    def trans_phi_rad2deg(self):
        self.phi_deg = np.rad2deg(self.phi)

    def trans_theta_deg2rad(self):
        self.theta = np.deg2rad(self.theta_deg)

    def trans_theta_rad2deg(self):
        self.theta_deg = np.rad2deg(self.theta)

    def trans_psi_deg2rad(self):
        self.psi = np.deg2rad(self.psi_deg)

    def trans_psi_rad2deg(self):
        self.psi_deg = np.rad2deg(self.psi)

    def trans_chi_deg2rad(self):
        self.chi = np.deg2rad(self.chi_deg)

    def trans_chi_rad2deg(self):
        self.chi_deg = np.rad2deg(self.chi)

    def trans_beta_deg2rad(self):
        self.beta = np.deg2rad(self.beta_deg)

    def trans_beta_rad2deg(self):
        self.beta_deg = np.rad2deg(self.beta)

    def trans_lon_deg2rad(self):
        self.lon = np.deg2rad(self.lon_deg)

    def trans_lon_rad2deg(self):
        self.lon_deg = np.rad2deg(self.lon)

    def trans_lat_deg2rad(self):
        self.lat = np.deg2rad(self.lat_deg)

    def trans_lat_rad2deg(self):
        self.lat_deg = np.rad2deg(self.lat)

    def trans_psi_cmd_rad2deg(self):
        self.psi_cmd_deg = np.rad2deg(self.psi_cmd)

    def trans_psi_cmd_deg2rad(self):
        self.psi_cmd = np.deg2rad(self.psi_cmd_deg)

    # --------------------------------------------------------------------------- #
    # Coordinate system transformation
    # --------------------------------------------------------------------------- #
    def set_mat_linear_BODY2NED(self):

        c_phi = np.cos(self.phi)
        c_theta = np.cos(self.theta)
        c_psi = np.cos(self.psi)
        s_phi = np.sin(self.phi)
        s_theta = np.sin(self.theta)
        s_psi = np.sin(self.psi)

        self.mat_linear_b2n = np.array(
            [[c_psi * c_theta, -s_psi * c_phi + c_psi * s_theta * s_phi, s_psi * s_phi + c_psi * c_phi * s_theta],
             [s_psi * c_theta, c_psi * c_phi + s_phi * s_theta * s_psi, -c_psi * s_phi + s_theta * s_theta * c_phi],
             [-s_theta, c_theta * s_phi, c_theta * c_phi]])

    def trans_linear_BODY2NED(self):
        self.set_mat_linear_BODY2NED()

        self.n_dot, self.e_dot, self.d_dot = \
            np.dot(self.mat_linear_b2n, np.array([[self.u], [self.v], [self.w]]), dtype=float)

    def set_mat_linear_NED2BODY(self):
        self.set_mat_linear_BODY2NED()
        self.mat_linear_n2b = self.mat_linear_b2n.T()

    def trans_linear_NED2BODY(self):
        self.u, self.v, self.w = \
            np.dot(self.mat_linear_n2b, np.array([[self.n_dot], [self.e_dot], [self.d_dot]]), dtype=float)

    def set_mat_angular_BODY2NED(self):
        c_phi = np.cos(self.phi)
        c_theta = np.cos(self.theta)
        s_phi = np.sin(self.phi)
        t_theta = np.tan(self.theta)

        if c_theta == 0:
            print("Transformation matrix is singular when theta = +-90 degrees.")

        self.mat_angular_b2n = np.array(
            [[1, s_phi * t_theta, c_phi * t_theta], [0, c_phi, -s_phi], [0, s_phi / c_theta, c_phi / c_theta]],
            dtype=float)

    def trans_angular_BODY2NED(self):
        self.phi_dot, self.theta_dot, self.psi_dot = np.dot(
            self.mat_angular_b2n, np.array([[self.p], [self.q], [self.r]]), dtype=float)

    def set_mat_angular_NED2BODY(self):
        self.set_mat_angular_BODY2NED()
        self.mat_angular_n2b = self.mat_angular_b2n.T()

    def trans_angular_NED2BODY(self):
        self.set_mat_angular_NED2BODY()
        self.p, self.q, self.r = np.dot(
            self.mat_angular_n2b, np.array([[self.phi_dot], [self.theta_dot], [self.psi_dot]]), dtype=float)

    def set_mat_BODY2NED(self):
        self.set_mat_linear_BODY2NED()
        self.set_mat_angular_BODY2NED()

        self.mat_b2n = np.vstack((np.hstack((self.mat_linear_b2n, np.zeros(
            (3, 3)))), np.hstack((np.zeros((3, 3)), self.mat_angular_b2n))))

    def set_mat_NED2BODY(self):
        self.set_mat_BODY2NED()
        self.mat_n2b = self.mat_b2n.T()

    def trans_MID_BODY2NED(self):
        self.set_mat_BODY2NED()
        self.n_dot, self.e_dot, self.d_dot, self.phi_dot, self.theta_dot, self.psi_dot = \
            np.dot(self.mat_b2n, np.array([[self.u], [self.v], [self.w], [self.p], [self.q], [self.r]], dtype=float))

    def trans_MID_NED2BODY(self):
        self.set_mat_NED2BODY()
        self.u, self.v, self.w, self.p, self.q, self.r = np.dot(
            self.mat_n2b,
            np.array(([self.n_dot], [self.e_dot], [self.d_dot], [self.phi_dot], [self.theta_dot], [self.psi_dot]),
                     dtype=float))

    def trans_COG_BODY2NED(self):
        self.set_mat_BODY2NED()
        self.nG_dot, self.eG_dot, self.dG_dot, self.phi_dot, self.theta_dot, self.psi_dot = \
            np.dot(self.mat_b2n, np.array([[self.uG], [self.vG], [self.wG], [self.p], [self.q], [self.r]], dtype=float))

    def trans_COG_NED2BODY(self):
        self.set_mat_NED2BODY()
        self.uG, self.vG, self.wG, self.p, self.q, self.r = np.dot(
            self.mat_n2b,
            np.array(([self.nG_dot], [self.eG_dot], [self.dG_dot], [self.phi_dot], [self.theta_dot], [self.psi_dot]),
                     dtype=float))

    def set_mat_NED2ECEF(self):
        # Get transformation matrix from NED coordinates to ECEF coordinates

        c_lon = np.cos(self.lon)
        s_lon = np.sin(self.lon)
        c_lat = np.cos(self.lat)
        s_lat = np.sin(self.lat)

        self.mat_linear_n2e = np.array(
            [[-c_lon * s_lat, -s_lon, -c_lon * c_lat], [-s_lon * s_lat, c_lon, -s_lon * c_lat], [c_lat, 0.0, -s_lat]],
            dtype=float)

    def trans_linear_NED2ECEF(self):
        self.set_mat_NED2ECEF()

        self.xE_dot, self.yE_dot, self.zE_dot = np.dot(self.mat_linear_n2e,
                                                       np.array((self.n_dot, self.e_dot, self.d_dot), dtype=float))

    def trans_ECEF2LLH(self, field):
        eps = 1
        tol = 1E-10

        self.lon = np.arctan2(self.yE, self.xE)
        p = np.sqrt(self.xE ** 2 + self.yE ** 2)
        self.lat = np.arctan(self.zE / (p * (1 - field.e ** 2)))
        self.hgt = 0.0

        while eps > tol:
            # Radius of curvature in the prime vertical
            N = field.r_e ** 2 / np.sqrt((field.r_e * np.cos(self.lat)) ** 2 + (field.r_p * np.sin(self.lat)) ** 2)
            h = p / np.cos(self.lat) - N
            lat0 = self.lat
            self.lat = np.arctan(self.zE / (p * (1 - field.e ** 2 * N / (N + h))))
            eps = abs(self.lat - lat0)
            self.hgt = self.zE / np.sin(self.lat) - N * (field.r_p / field.r_e) ** 2

    def trans_LLH2ECEF(self, field):

        N = field.r_e ** 2 / np.sqrt((field.r_e * np.cos(self.lat)) ** 2 + (field.r_p * np.sin(self.lat)) ** 2)

        self.xE = (N + self.hgt) * np.cos(self.lat) * np.cos(self.lon)
        self.yE = (N + self.hgt) * np.cos(self.lat) * np.sin(self.lon)
        self.zE = (N * (field.r_p / field.r_e) ** 2 + self.hgt) * np.sin(self.lat)

    def trans_linear_MID2COG(self):
        self.uG = self.u + self.hull.zG * self.q - self.hull.yG * self.r
        self.vG = self.v + self.hull.xG * self.r - self.hull.yG * self.p
        self.wG = self.w + self.hull.yG * self.p - self.hull.xG * self.q

    def set_U(self):
        # Get ship speed relative to water
        self.U = np.sqrt(self.u ** 2 + self.v ** 2)

    def set_beta(self):
        # Get ship drift angle
        self.beta = np.arctan(-self.v / self.u)

    def set_chi(self):
        # Get ship course angle
        self.chi = self.psi - self.beta

    def set_Re(self, field):
        self.Re = field.water.rho * self.U * self.hull.LPP / field.water.mu

    def set_Fr(self, field):
        """
        Get the Froude number.
        Based on the Froude number, a marine craft can be classed as:
        Displacement vessels (Fr < 0.4) for common ships where buoyancy force dominates relative to the hydrodynamic forces
        Semi-displacement hull (0.4~0.5 < Fr < 1.0~1.2) where buoyancy force is not dominant at the maximum speed
        Planning hull (Fr > 1.0~1.2) where hydrodynamic force mainly carries the weight.
        """
        self.Fr = self.U / np.sqrt(field.g * self.hull.LPP)

    # --------------------------------------------------------------------------- #
    # Dynamic non-dimensional factors
    # --------------------------------------------------------------------------- #
    """
    Using SNAME1950 for prime system I:
    SNAME1950-Nomenclature for treating the motion of a submerged body through a fluid

    Using Yasukawa2015 for prime system II
    Yasukawa2015-Introduction of {MMG} standard method for ship maneuvering predictions
    """

    def set_pri_time(self, hull):
        # Return non-dimensional time factor using prime system I and II
        self.pri_time = hull.LPP / self.U

    def set_pri_linear_speed(self):
        # Return non-dimensional linear speed factor using prime system I and II
        self.pri_linear_speed = self.U

    def set_pri_angular_speed(self, hull):
        # Return non-dimensional angular speed factor using prime system I and II
        self.pri_angular_speed = self.U / hull.LPP

    def set_pri_linear_acc(self, hull):
        # Return non-dimensional linear acceleration factor using prime system I and II
        self.pri_linear_acc = self.U ** 2 / hull.LPP

    def set_pri_angular_acc(self, hull):
        # Return non-dimensional angular acceleration factor using prime system I and II
        self.pri_angular_acc = self.U ** 2 / hull.LPP ** 2

    def set_pri_I_force(self, field, hull):
        # Return non-dimensional force factor using prime system I
        self.pri_I_force = 0.5 * field.water.rho * self.U ** 2 * hull.LPP ** 2

    def set_pri_II_force(self, field, hull):
        # Return non-dimensional force factor using prime system II
        self.pri_II_force = 0.5 * field.water.rho * self.U ** 2 * hull.LPP * hull.TMS

    def set_pri_I_moment(self, field, hull):
        # Return non-dimensional moment factor using prime system I
        self.pri_I_moment = 0.5 * field.water.rho * self.U ** 2 * hull.LPP ** 3

    def set_pri_II_moment(self, field, hull):
        # Return non-dimensional moment factor using prime system II
        self.pri_II_moment = 0.5 * field.water.rho * self.U ** 2 * hull.LPP ** 2 * hull.TMS

    """
    Using Norrbin1970 for bis system:
    Norrbin1970-Theory and observation on the use of a mathematical model for ship maneuvering in deep and confined waters

    For the bias system, the body density ratio mu = m/(rho*V), where m is the mass unit and V is the hull contour displacement.
    mu varies with different types of vehicles as follows:
    mu < 1: Under vehicles (ROVs, AUVs and submarines)
    mu = 1: Floating ships/rigs and neutrally buoyant underwater vehicles
    mu > 1: Heavy torpedoes (typically mu = 1.3-1.5)
    """

    def set_bis_time(self, field, hull):
        # Return non-dimensional time factor using bis system
        self.bis_time = np.sqrt(hull.LPP / field.g)

    def set_bis_linear_speed(self, field, hull):
        # Return non-dimensional linear speed factor using bis system
        self.bis_linear_speed = np.sqrt(hull.LPP * field.g)

    def set_bis_angular_speed(self, field, hull):
        # Return non-dimensional angular speed factor using bis system
        self.bis_angular_speed = np.sqrt(field.g / hull.LPP)

    def set_bis_linear_acc(self, field):
        # Return non-dimensional linear acceleration factor using bis system
        self.bis_linear_acc = field.g

    def set_bis_angular_acc(self, field, hull):
        # Return non-dimensional angular acceleration factor using bis system
        self.bis_angular_acc = field.g / hull.LPP

    def set_bis_force(self, field, hull, mu=1.0):
        # Return non-dimensional force factor using bis system
        self.bis_force = mu * field.water.rho * field.g * hull.DISPV

    def set_bis_moment(self, field, hull, mu=1.0):
        # Return non-dimensional moment factor using bis system
        self.bis_moment = mu * field.water.rho * field.g * hull.DISPV * hull.LPP

    def __str__(self):
        return "\n".join(["%s: %s" % item for item in self.__dict__.items()])
