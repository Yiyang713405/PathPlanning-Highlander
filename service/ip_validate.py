# -*- coding: utf-8 -*-
"""
Provide pkg_validation data for simulations.
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
def provide_validation(vessel, maneuver, scale="full-scale"):
    folder_path = os.path.dirname(os.path.realpath(__file__))

    val_hist_deg = None
    val_prefix = None
    val_suffix = None

    if scale == "full-scale":
        val_suffix = "FS"
    elif scale == "model-scale":
        val_suffix = "MS"
    else:
        ValueError(scale, "is not properly defined!")

    if vessel.vessel_name in ["KVLCC2_MARIN_L7"]:
        if maneuver.maneuver_type == "turning":
            if maneuver.delta_cmd_deg == 35:
                val_prefix = "\\pkg_validation\\SIMMAN2008_tanker_KVLCC2_MARIN_turning_star_35"
            elif maneuver.delta_cmd_deg == -35:
                val_prefix = "\\pkg_validation\\SIMMAN2008_tanker_KVLCC2_MARIN_turning_port_35"

        elif maneuver.maneuver_type == "zigzag":
            if maneuver.delta_cmd_deg == 20 and maneuver.psi_cmd_deg == 20:
                val_prefix = "\\pkg_validation\\SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_star_20"
            elif maneuver.delta_cmd_deg == 10 and maneuver.psi_cmd_deg == 10:
                val_prefix = "\\pkg_validation\\SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_star_10"
            elif maneuver.delta_cmd_deg == -20 and maneuver.psi_cmd_deg == -20:
                val_prefix = "\\pkg_validation\\SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_port_20"
            elif maneuver.delta_cmd_deg == -10 and maneuver.psi_cmd_deg == -10:
                val_prefix = "\\pkg_validation\\SIMMAN2008_tanker_KVLCC2_MARIN_zigzag_port_10"

    elif vessel.vessel_name in ["KCS_MARIN_L6", "KCS_NMRI_L3"]:
        if maneuver.maneuver_type == "turning":
            if maneuver.delta_cmd_deg == 35:
                val_prefix = "\\pkg_validation\\SIMMAN2020_container_KCS_MARIN_turning_star_35"
            elif maneuver.delta_cmd_deg == -35:
                val_prefix = "\\pkg_validation\\SIMMAN2020_container_KCS_MARIN_turning_port_35"

        elif maneuver.maneuver_type == "zigzag":
            if maneuver.delta_cmd_deg == 20 and maneuver.psi_cmd_deg == 20:
                val_prefix = "\\pkg_validation\\SIMMAN2020_container_KCS_MARIN_zigzag_star_20"
            elif maneuver.delta_cmd_deg == 10 and maneuver.psi_cmd_deg == 10:
                val_prefix = "\\pkg_validation\\SIMMAN2020_container_KCS_MARIN_zigzag_star_10"
            elif maneuver.delta_cmd_deg == -20 and maneuver.psi_cmd_deg == -20:
                val_prefix = "\\pkg_validation\\SIMMAN2020_container_KCS_MARIN_zigzag_port_20"
            elif maneuver.delta_cmd_deg == -10 and maneuver.psi_cmd_deg == -10:
                val_prefix = "\\pkg_validation\\SIMMAN2020_container_KCS_MARIN_zigzag_port_10"

    elif vessel.vessel_name in ["bulk_6700t_CCS_L4"]:
        if maneuver.maneuver_type == "turning":
            if maneuver.delta_cmd_deg == 35:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_turning_star_35"
            elif maneuver.delta_cmd_deg == 25:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_turning_star_25"
            elif maneuver.delta_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_turning_star_15"
            elif maneuver.delta_cmd_deg == -35:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_turning_port_35"
            elif maneuver.delta_cmd_deg == -25:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_turning_port_25"
            elif maneuver.delta_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_turning_port_15"

        elif maneuver.maneuver_type == "zigzag":
            if maneuver.delta_cmd_deg == 20 and maneuver.psi_cmd_deg == 20:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_zigzag_star_20"
            elif maneuver.delta_cmd_deg == 15 and maneuver.psi_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_zigzag_star_15"
            elif maneuver.delta_cmd_deg == 10 and maneuver.psi_cmd_deg == 10:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_zigzag_star_10"
            elif maneuver.delta_cmd_deg == -20 and maneuver.psi_cmd_deg == -20:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_zigzag_port_20"
            elif maneuver.delta_cmd_deg == -15 and maneuver.psi_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_zigzag_port_15"
            elif maneuver.delta_cmd_deg == -10 and maneuver.psi_cmd_deg == -10:
                val_prefix = "\\pkg_validation\\bulk_6700t_CCS_zigzag_port_10"

    elif vessel.vessel_name in ["tanker_3500t_CCS_L4"]:
        if maneuver.maneuver_type == "turning":
            if maneuver.delta_cmd_deg == 35:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_turning_star_35"
            elif maneuver.delta_cmd_deg == 25:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_turning_star_25"
            elif maneuver.delta_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_turning_star_15"
            elif maneuver.delta_cmd_deg == -35:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_turning_port_35"
            elif maneuver.delta_cmd_deg == -25:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_turning_port_25"
            elif maneuver.delta_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_turning_port_15"

        elif maneuver.maneuver_type == "zigzag":
            if maneuver.delta_cmd_deg == 20 and maneuver.psi_cmd_deg == 20:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_zigzag_star_20"
            elif maneuver.delta_cmd_deg == 15 and maneuver.psi_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_zigzag_star_15"
            elif maneuver.delta_cmd_deg == 10 and maneuver.psi_cmd_deg == 10:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_zigzag_star_10"
            elif maneuver.delta_cmd_deg == -20 and maneuver.psi_cmd_deg == -20:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_zigzag_port_20"
            elif maneuver.delta_cmd_deg == -15 and maneuver.psi_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_zigzag_port_15"
            elif maneuver.delta_cmd_deg == -10 and maneuver.psi_cmd_deg == -10:
                val_prefix = "\\pkg_validation\\tanker_3500t_CCS_zigzag_port_10"

    elif vessel.vessel_name in ["YIHUA_Container_64_TEU_L4"]:
        if maneuver.maneuver_type == "turning":
            if maneuver.delta_cmd_deg == 35:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_star_35"
            elif maneuver.delta_cmd_deg == 30:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_star_30"
            elif maneuver.delta_cmd_deg == 25:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_star_25"
            elif maneuver.delta_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_star_15"
            elif maneuver.delta_cmd_deg == -35:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_port_35"
            elif maneuver.delta_cmd_deg == -30:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_port_30"
            elif maneuver.delta_cmd_deg == -25:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_port_25"
            elif maneuver.delta_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_turning_port_15"

        elif maneuver.maneuver_type == "zigzag":
            if maneuver.delta_cmd_deg == 20 and maneuver.psi_cmd_deg == 20:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_star_20"
            elif maneuver.delta_cmd_deg == 15 and maneuver.psi_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_star_15"
            elif maneuver.delta_cmd_deg == 10 and maneuver.psi_cmd_deg == 10:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_star_10"
            elif maneuver.delta_cmd_deg == -20 and maneuver.psi_cmd_deg == -20:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_port_20"
            elif maneuver.delta_cmd_deg == -15 and maneuver.psi_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_port_15"
            elif maneuver.delta_cmd_deg == -10 and maneuver.psi_cmd_deg == -10:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_port_10"

    elif vessel.vessel_name in ["QIANHANG_Container_64_TEU_L4"]:
        if maneuver.maneuver_type == "turning":
            if maneuver.delta_cmd_deg == 35:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_star_35"
            elif maneuver.delta_cmd_deg == 30:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_star_30"
            elif maneuver.delta_cmd_deg == 25:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_star_25"
            elif maneuver.delta_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_star_15"
            elif maneuver.delta_cmd_deg == -35:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_port_35"
            elif maneuver.delta_cmd_deg == -30:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_port_30"
            elif maneuver.delta_cmd_deg == -25:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_port_25"
            elif maneuver.delta_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\QIANHANG_container_64TEU_turning_port_15"

        elif maneuver.maneuver_type == "zigzag":
            if maneuver.delta_cmd_deg == 20 and maneuver.psi_cmd_deg == 20:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_star_20"
            elif maneuver.delta_cmd_deg == 15 and maneuver.psi_cmd_deg == 15:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_star_15"
            elif maneuver.delta_cmd_deg == 10 and maneuver.psi_cmd_deg == 10:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_star_10"
            elif maneuver.delta_cmd_deg == -20 and maneuver.psi_cmd_deg == -20:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_port_20"
            elif maneuver.delta_cmd_deg == -15 and maneuver.psi_cmd_deg == -15:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_port_15"
            elif maneuver.delta_cmd_deg == -10 and maneuver.psi_cmd_deg == -10:
                val_prefix = "\\pkg_validation\\YIHUA_container_64TEU_zigzag_port_10"

    else:
        ValueError("No validation data for", str(vessel.vessel_name), "is available!")

    file_path = folder_path + val_prefix + "_" + val_suffix + ".txt"
    print(file_path)

    if file_path is not None:
        val_hist_deg = []
        with open(file_path, encoding="utf-8") as f:
            f.readline()
            f.readline()
            for line in f:
                items = [float(x) for x in line.split()]
                val_hist_deg.append(items)
        val_hist_deg = np.array(val_hist_deg)

    return val_hist_deg
