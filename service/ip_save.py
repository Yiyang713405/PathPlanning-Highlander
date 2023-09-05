# -*- coding: utf-8 -*-
"""
Save simulation data and plots.
"""

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import os

import numpy as np

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2020/3/1"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2020 Jialun Liu. All rights reserved."


# --------------------------------------------------------------------------- #
# Function
# --------------------------------------------------------------------------- #
def save_hist(sim_hist, file_name):
    header_text = "{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}\n" \
                  "{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}".format("time", "n", "u", "e", "v", "psi",
                                                                                  "r", "rps", "delta",
                                                                                  "(s)", "(m)", "(m/s)", "(m)", "(m/s)",
                                                                                  "(deg)", "(deg/s)", "(Hz)", "(deg)")
    folder_path = os.path.dirname(os.path.realpath(__file__))

    np.savetxt(folder_path + "\\pkg_result\\" + file_name + ".txt", sim_hist, fmt="%-10.3f", delimiter="",
               header=header_text, comments="")


def save_force(field_param, ship_param, sim_hist):
    X_H = np.zeros((sim_hist.shape[0], 1))
    Y_hull = np.zeros((sim_hist.shape[0], 1))
    Z_hull = np.zeros((sim_hist.shape[0], 1))
    K_hull = np.zeros((sim_hist.shape[0], 1))
    M_hull = np.zeros((sim_hist.shape[0], 1))
    N_hull = np.zeros((sim_hist.shape[0], 1))
    X_prop = np.zeros((sim_hist.shape[0], 1))
    Y_prop = np.zeros((sim_hist.shape[0], 1))
    Z_prop = np.zeros((sim_hist.shape[0], 1))
    K_prop = np.zeros((sim_hist.shape[0], 1))
    M_prop = np.zeros((sim_hist.shape[0], 1))
    N_prop = np.zeros((sim_hist.shape[0], 1))
    X_rudd = np.zeros((sim_hist.shape[0], 1))
    Y_rudd = np.zeros((sim_hist.shape[0], 1))
    Z_rudd = np.zeros((sim_hist.shape[0], 1))
    K_rudd = np.zeros((sim_hist.shape[0], 1))
    M_rudd = np.zeros((sim_hist.shape[0], 1))
    N_rudd = np.zeros((sim_hist.shape[0], 1))
    X_total = np.zeros((sim_hist.shape[0], 1))
    Y_total = np.zeros((sim_hist.shape[0], 1))
    Z_total = np.zeros((sim_hist.shape[0], 1))
    M_total = np.zeros((sim_hist.shape[0], 1))
    K_total = np.zeros((sim_hist.shape[0], 1))
    N_total = np.zeros((sim_hist.shape[0], 1))

    for i_step in range(sim_hist.shape[0]):
        # Calculate forces at each time step
        X_H[i_step], Y_hull[i_step], Z_hull[i_step], \
        K_hull[i_step], M_hull[i_step], N_hull[i_step], \
        X_prop[i_step], Y_prop[i_step], Z_prop[i_step], \
        K_prop[i_step], M_prop[i_step], N_prop[i_step], \
        X_rudd[i_step], Y_rudd[i_step], Z_rudd[i_step], \
        K_rudd[i_step], M_rudd[i_step], N_rudd[i_step] = \
            ship_param.get_total_force(sim_hist[i_step], field_param)

        # Total surge force on body along body axis x
        X_total[i_step] = X_H[i_step] + X_prop[i_step] + X_rudd[i_step]
        # Total sway force on body along body axis y
        Y_total[i_step] = Y_hull[i_step] + Y_prop[i_step] + Y_rudd[i_step]
        # Total heave force on body along body axis z
        Z_total[i_step] = Z_hull[i_step] + Z_prop[i_step] + Z_rudd[i_step]
        # Total roll moment on body around body axis x
        M_total[i_step] = K_hull[i_step] + K_prop[i_step] + K_rudd[i_step]
        # Total pitch moment on body around body axis y
        K_total[i_step] = M_hull[i_step] + M_prop[i_step] + M_rudd[i_step]
        # Total yaw moment on body around body axis z
        N_total[i_step] = N_hull[i_step] + N_prop[i_step] + N_rudd[i_step]

# ----------------------------------------------------------------------- #
# Save Output
# ----------------------------------------------------------------------- #
# if is_output_save:
#     output_rec = hstack((tHist.reshape((tHist.size, 1)), delta_deg_rec.reshape(
#         (delta_deg_rec.size, 1)), yHist, X_H_rec.reshape((X_H_rec.size, 1)), Y_H_rec.reshape(
#         (Y_H_rec.size, 1)), N_H_rec.reshape((N_H_rec.size, 1)), X_P_rec.reshape(
#         (X_P_rec.size, 1)), Y_P_rec.reshape((X_P_rec.size, 1)), N_P_rec.reshape(
#         (X_P_rec.size, 1)), u_R_rec.reshape((u_R_rec.size, 1)), v_R_rec.reshape(
#         (v_R_rec.size, 1)), U_R_rec.reshape((U_R_rec.size, 1)), Re_R_rec.reshape(
#         (Re_R_rec.size, 1)), X_R_rec.reshape(
#         (X_R_rec.size, 1)), Y_R_rec.reshape(
#         (Y_R_rec.size, 1)), N_R_rec.reshape(
#         (N_R_rec.size, 1)), XTotal_rec.reshape(
#         (XTotal_rec.size, 1)), YTotal_rec.reshape(
#         (YTotal_rec.size, 1)), NTotal_rec.reshape((NTotal_rec.size, 1))))
#
#     np.savetxt(
#         file_name + "_output.csv",
#         output_rec,
#         delimiter=",",
#         header="time (s), delta (deg), "
#                "x0 (m), u (m/s), "
#                "y0 (m), v (m/s), "
#                "psi (rad), r (rad/s), "
#                "X_H (N), Y_H (N), N_H (Nm), "
#                "X_P (N), Y_P (N), N_P (Nm), "
#                "u_R (m/s), v_R (m/s), U_R (m/s), "
#                "Re_R (1), "
#                "X_R (N), Y_R (N), N_R (Nm), "
#                "X_total (N), Y_total (N), N_total (Nm)")
# ----------------------------------------------------------------------- #
# Manoeuvre Information
# ----------------------------------------------------------------------- #
# if maneuver.category == "turning":
#     if maneuver.direction == "star":
#         AD_index = (abs(yHist[:, 4] - pi / 2)).argmin()
#         TR_index = AD_index
#         TD_index = (abs(yHist[:, 4] - pi)).argmin()
#
#         advance_AD = (
#             yHist[:, 0, AD_index] -
#             yHist[:, 0, sim_time.sim_delay]) / ship.hull_KVLCC2_L7_MARIN.LPP
#         transfer_TR = (
#             yHist[:, 2, TR_index] -
#             yHist[:, 2, sim_time.sim_delay]) / ship.hull_KVLCC2_L7_MARIN.LPP
#         tactical_diameter_TD = (
#             yHist[:, 2, TD_index] -
#             yHist[:, 2, sim_time.sim_delay]) / ship.hull_KVLCC2_L7_MARIN.LPP
#         print("""
#         Advance:           {AD} (1)    IMO: 4.5
#         Transfer:          {TR} (1)    IMO: None
#         Tactical diameter: {TD} (1)    IMO: 5.0
#         """.format(AD=advance_AD,
#                    TR=transfer_TR,
#                    TD=tactical_diameter_TD)
#         if is_benchmark:
#             AD_MARIN = abs(benchmark[:, 6] - benchmark[0, 6]
#                               - pi / 2).argmin()
#             TR_MARIN = AD_MARIN
#             TD_MARIN = abs(benchmark[:, 6] - benchmark[0, 6]
#                               - pi).argmin()
#             print("""
#         Benchmark: MARIN
#         Advance:           {AD} (1)    IMO: 4.5
#         Transfer:          {TR} (1)    IMO: None
#         Tactical diameter: {TD} (1)    IMO: 5.0
#             """.format(AD=(benchmark[AD_MARIN, 1] +
#                            benchmark[0, 1]) / 320,
#                        TR=(benchmark[TR_MARIN, 2] -
#                            benchmark[0, 2]) / 320,
#                        TD=(benchmark[TD_MARIN, 2] +
#                            benchmark[0, 2]) / 320))
#     elif maneuver.direction == "port":
#         AD_index = (abs(yHist[:, 4] + pi / 2)).argmin()
#         TR_index = AD_index
#         TD_index = (abs(yHist[:, 4] + pi)).argmin()
#
#         advance_AD = (
#             yHist[:, 0, AD_index] -
#             yHist[:, 0, sim_time.sim_delay]) / ship.hull_KVLCC2_L7_MARIN.LPP
#         transfer_TR = (
#             yHist[:, 2, TR_index] -
#             yHist[:, 2, sim_time.sim_delay]) / ship.hull_KVLCC2_L7_MARIN.LPP
#         tactical_diameter_TD = (
#             yHist[:, 2, TD_index] -
#             yHist[:, 2, sim_time.sim_delay]) / ship.hull_KVLCC2_L7_MARIN.LPP
#         print("""
#         Advance:           {AD} (1)    IMO: 4.5
#         Transfer:          {TR} (1)    IMO: None
#         Tactical diameter: {TD} (1)    IMO: 5.0
#         """.format(AD=advance_AD,
#                    TR=transfer_TR,
#                    TD=tactical_diameter_TD))
#         if is_benchmark:
#             AD_MARIN = abs(benchmark[:, 6] - benchmark[0, 6]
#                               + pi / 2).argmin()
#             TR_MARIN = AD_MARIN
#             TD_MARIN = abs(benchmark[:, 6] - benchmark[0, 6]
#                               + pi).argmin()
#             print("""
#         Benchmark: MARIN
#         Advance:           {AD} (1)    IMO: 4.5
#         Transfer:          {TR} (1)    IMO: None
#         Tactical diameter: {TD} (1)    IMO: 5.0
#             """.format(AD=(benchmark[AD_MARIN, 1] -
#                            benchmark[0, 1]) / 320,
#                        TR=(benchmark[TR_MARIN, 2] -
#                            benchmark[0, 2]) / 320,
#                        TD=(benchmark[TD_MARIN, 2] -
#                            benchmark[0, 2]) / 320))
#
# if maneuver.category == "zigzag":
#     if maneuver.direction == "star":
#         # first_overshoot_angle = degrees(yHist[:, 4].max()) - \
#         #     maneuver.max_psi_deg
#         # second_overshoot_angle = degrees(yHist[:, 4].min()) + \
#         #     maneuver.max_psi_deg
#
#         FO_index = where(yHist[:, 4] == yHist[:, 4].max())
#         first_overshoot_angle = degrees(yHist[:, 4, FO_index]) - \
#             maneuver.max_psi_deg
#         first_overshoot_time = 0
#
#         SO_index = where(yHist[:, 4] == yHist[:, 4].min())
#         second_overshoot_angle = abs(degrees(yHist[:, 4, SO_index])
#                                      + maneuver.max_psi_deg)
#         second_overshoot_time = 0
#
#         print("""
#         First overshoot angle: {FOD} (deg)
#         First overshoot time:  {FOT} (s)
#         Second overshoot angle: {SOD} (deg)
#         Second overshoot time:  {SOT} (s)
#         """.format(FOD=first_overshoot_angle[0],
#                    FOT=first_overshoot_time,
#                    SOD=second_overshoot_angle[0],
#                    SOT=second_overshoot_time))
#     elif maneuver.direction == "port":
#         FO_index = where(yHist[:, 4] == yHist[:, 4].max())
#         first_overshoot_angle = degrees(yHist[:, 4, FO_index]) - \
#             maneuver.max_psi_deg
#         first_overshoot_time = 0
#
#         SO_index = where(yHist[:, 4] == yHist[:, 4].min())
#         second_overshoot_angle = abs(degrees(yHist[:, 4, SO_index])
#                                      + maneuver.max_psi_deg)
#         second_overshoot_time = 0
#
#         print("""
#         First overshoot angle: {FOD} (deg)
#         First overshoot time:  {FOT} (s)
#         Second overshoot angle: {SOD} (deg)
#         Second overshoot time:  {SOT} (s)
#         """.format(FOD=first_overshoot_angle[0],
#                    FOT=first_overshoot_time,
#                    SOD=second_overshoot_angle[0],
#                    SOT=second_overshoot_time))
