# -*- coding: utf-8 -*-
"""
Field defines the nature environment of a rho including the following classes:
    - Water:
    - Current:
    - Wave:
    - Swell:
    - Air:
    - Wind:
    - Cloud:
    - Fog:
"""

import numpy as np

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2018/7/15"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2018 Jialun Liu. All rights reserved."


# --------------------------------------------------------------------------- #
# Water
# --------------------------------------------------------------------------- #
class Water(object):
    """
    Water defines water properties.
    """

    def __init__(self, name, temperature, density, depth):
        self.name = name  # (-) A string
        self.t = temperature  # (Celsius degree) Temperature of water
        self.rho = density  # (kg/m^3) Mass density of water
        self.h = depth  # (m) Water depth
        self.mu = 0.001003  # (Ns/m^2) Dynamic viscosity of water
        self.niu = self.mu / self.rho  # (m^2/s) Kinematic viscosity of water

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


fresh_water = Water("fresh water", 20.0, 1000.0, 10.0)
salt_water = Water("salt water", 20.0, 1025.0, 100.0)


# --------------------------------------------------------------------------- #
# Current
# --------------------------------------------------------------------------- #
class Current(object):
    """
    Current defines water properties.
    """

    def __init__(self, name, direction_deg, speed):
        self.name = name  # (-) A string
        self.psi_deg = direction_deg  # (deg) Course of current velocity in degree
        self.psi = np.deg2rad(direction_deg)  # (rad) Course of current velocity in radian
        self.U = speed  # (m/s) Current velocity

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


no_current = Current("no current", 0.0, 0.0)


# --------------------------------------------------------------------------- #
# Wave
# --------------------------------------------------------------------------- #
class Wave(object):
    """
    Wave defines wave properties.
    """

    def __init__(self, name):
        self.name = name  # (-) A string

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


no_wave = Wave("no wave")


# --------------------------------------------------------------------------- #
# Swell 涌浪
# --------------------------------------------------------------------------- #
class Swell(object):
    """
    Swell defines wave properties.
    """

    def __init__(self, name):
        self.name = name  # (-) A string

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


no_swell = Swell("no swell")


# --------------------------------------------------------------------------- #
# Air
# --------------------------------------------------------------------------- #
class Air(object):
    """
    Air defines air properties.
    """

    def __init__(self, name, temperature, density):
        self.name = name  # (-) A string
        self.t = temperature  # (Celsius degree) Temperature of air
        self.p = None  # (Pa) Pressure
        self.h = None  # (-) Humidity
        self.rho = density  # (kg/m^3) Mass density of air

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


fresh_air = Air("fresh air", 20, 1.29)


# --------------------------------------------------------------------------- #
# Wind
# --------------------------------------------------------------------------- #
class Wind(object):
    """
    Wind defins wind properties.
    """

    def __init__(self, name, scale):
        self.name = name  # (-) A string
        self.scale = scale  # (-) Beaufort wind scale in a range of 0 to 16

        if int(scale) == 0:
            self.U_min = 0.0  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 0.2  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 1:
            self.U_min = 0.3  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 1.5  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 2:
            self.U_min = 1.6  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 3.3  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 3:
            self.U_min = 3.4  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 5.4  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 4:
            self.U_min = 5.5  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 7.9  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 5:
            self.U_min = 8.0  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 10.7  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 6:
            self.U_min = 10.8  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 13.8  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 7:
            self.U_min = 13.9  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 17.1  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 8:
            self.U_min = 17.2  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 20.7  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 9:
            self.U_min = 20.8  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 24.4  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 10:
            self.U_min = 24.5  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 28.4  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 11:
            self.U_min = 28.5  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 32.6  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 12:
            self.U_min = 32.7  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 36.9  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 13:
            self.U_min = 37.0  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 41.4  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 14:
            self.U_min = 41.5  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 46.1  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        elif int(scale) == 15:
            self.U_max = 46.2  # (m/s) Maximum wind speed at certain Beaufort wind scale.
            self.U_min = 50.9  # (m/s) Minimum wind speed at certain Beaufort wind scale

        elif int(scale) == 16:
            self.U_min = 51.0  # (m/s) Minimum wind speed at certain Beaufort wind scale.
            self.U_max = 56.0  # (m/s) Maximum wind speed at certain Beaufort wind scale.

        self.U_avg = (self.U_max + self.U_min) / 2.0  # (m/s)

        self.psi_A_deg = None  # (deg) Course of apparent/relative wind direction in degree
        self.psi_A = None  # (rad) Course of apparent/relative wind direction in radian
        self.psi_T_deg = None  # # (deg) Course of true/absolute wind direction in degree
        self.psi_T = None  # (rad) Course of true/absolute wind direction in radian

        self.U_A = None  # (m/s) Apparent/relative wind speed
        self.U_T = None  # (m/s) True/absolute wind speed

    def set_psi_A_deg(self, apparent_direction_deg):
        self.psi_A_deg = apparent_direction_deg  # (deg) Course of apparent/relative wind direction in degree

    def set_psi_A(self):
        self.psi_A = np.deg2rad(self.psi_A_deg)  # (rad) Course of apparent/relative wind direction in radian

    def set_psi_T_deg(self, true_direction_deg):
        self.psi_T_deg = true_direction_deg  # # (deg) Course of true/absolute wind direction in degree

    def set_psi_T(self):
        self.psi_T = np.deg2rad(self.psi_T_deg)  # (rad) Course of true/absolute wind direction in radian

    def set_U_A(self, apparent_speed):
        self.U_A = apparent_speed  # (m/s) Apparent/relative wind speed

    def get_U_T(self, true_speed):
        self.U_T = true_speed  # (m/s) True/absolute wind speed

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


W0_calm = Wind("calm", 0)
W1_light_air = Wind("light_air", 1)
W2_light_breeze = Wind("light_breeze", 2)
W3_gentle_breeze = Wind("gentle_breeze", 3)
W4_moderate_breeze = Wind("moderate_breeze", 4)
W5_fresh_breeze = Wind("fresh_breeze", 5)
W6_strong_breeze = Wind("strong_breeze", 6)
W7_moderate_gale = Wind("moderate_gale", 7)
W8_fresh_gale = Wind("fresh_gale", 8)
W9_strong_gale = Wind("strong_gale", 9)
W10_whole_gale = Wind("whole_gale", 10)
W11_storm = Wind("storm", 11)
W12_hurricane = Wind("hurricane", 12)
W13_typhoon = Wind("typhon", 13)
W14_strong_typhoon = Wind("strong_typhoon", 14)
W15_strong_typhoon = Wind("strong_typhoon", 15)
W16_super_typhoon = Wind("super_typhoon", 16)


# --------------------------------------------------------------------------- #
# Cloud
# --------------------------------------------------------------------------- #
class Cloud(object):

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Fog
# --------------------------------------------------------------------------- #
class Fog(object):

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Obstacle
# --------------------------------------------------------------------------- #
class Obstacle(object):

    def __init__(self):
        self.lon = None  # (deg) Longitude
        self.lat = None  # (deg) Latitude
        self.xE = None  # (m) Coordinate in x direction of the ECEF coordinate system
        self.yE = None  # (m) Coordinate in y direction of the ECEF coordinate system
        self.n = None  # (m) Coordinate in n direction of the NED coordinate system
        self.e = None  # (m) Coordinate in e direction of the NED coordinate system

        self.U = None  # Speed
        self.psi = None  # Heading angle

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Field
# --------------------------------------------------------------------------- #
class Field(object):
    """"Class Field defines the particulars of the Field,
    including water, wind, current and swell."""

    def __init__(self, name, water, air):
        self.name = name  # (-) A string

        # Geographic parameters
        self.g = 9.80665  # (m/s^2) Gravity acceleration
        self.r_e = 6378137  # (m) Equatorial radius of ellipsoid (semi-major axis)
        self.r_p = 6356752.3142  # (m) Polar axis radius of ellipsoid (semi-minor axis)
        self.w_e = 7.292115E-5  # (rad/s) Angular velocity of the earth
        self.e = 0.08181979099211  # (1) Eccentricity of ellipsoid

        # Field components
        self.water = water  # An instance of Water
        self.current = None  # An instance of Current
        self.wave = None  # An instance of Wave
        self.swell = None  # An instance of Swell
        self.air = air  # An instance of Air
        self.wind = None  # An instance of Wind
        self.cloud = None  # An instance of Cloud
        self.fog = None  # An instance of Fog

    def set_water(self, water):
        self.water = water

    def set_current(self, current):
        self.current = current

    def set_wave(self, wave):
        self.wave = wave

    def set_swell(self, swell):
        self.swell = swell

    def set_air(self, air):
        self.air = air

    def set_wind(self, wind):
        self.wind = wind

    def set_cloud(self, cloud):
        self.cloud = cloud

    def set_fog(self, fog):
        self.fog = fog

    def __str__(self):
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])


OpenSea = Field("open sea", salt_water, fresh_air)
OpenLake = Field("open lake", fresh_water, fresh_air)
