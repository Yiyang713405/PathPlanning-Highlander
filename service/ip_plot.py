# -*- coding: utf-8 -*-
"""
Define all the plot functions of figures.
"""
# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import matplotlib.pyplot as plt

import numpy as np

plt.rcParams["lines.linewidth"] = 1.5
plt.rcParams["savefig.dpi"] = 600
# plt.rcParams["figure.dpi"] = 300

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
def plot_path_ms_non(fig, axes, vessel, sim_hist_ms, val_hist=None):
    axes.cla()
    n_hist = sim_hist_ms[:, vessel.hist_idx_n]  # n_hist (m)
    e_hist = sim_hist_ms[:, vessel.hist_idx_e]  # e_hist (m)
    axes.plot(e_hist / vessel.LPP, n_hist / vessel.LPP, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        n_val = val_hist[:, vessel.val_idx_n]  # n_hist (m)
        e_val = val_hist[:, vessel.val_idx_e]  # e_hist (m)
        axes.plot(
            e_val / vessel.LPP / vessel.SC,
            n_val / vessel.LPP / vessel.SC,
            color="r",
            linestyle="--",
            label="EXP")

    axes.set_xlabel(r"$y_0/L_{pp}$ (-)")
    axes.set_ylabel(r"$x_0/L_{pp}$ (-)")
    axes.legend(loc="best")
    axes.axis("equal")
    axes.grid(True)

    fig.tight_layout()


def plot_path_fs_non(fig, axes, vessel, sim_hist=None, val_hist=None, scale="full-scale"):
    axes.cla()
    n_hist = sim_hist[:, vessel.hist_idx_n]  # n_hist (m)
    e_hist = sim_hist[:, vessel.hist_idx_e]  # e_hist (m)
    if scale == "full-scale":
        axes.plot(
            e_hist / vessel.hull.LPP / vessel.SC,
            n_hist / vessel.hull.LPP / vessel.SC,
            color="b",
            linestyle="-",
            label="SIM")
    elif scale == "model-scale":
        axes.plot(
            e_hist / vessel.hull.LPP,
            n_hist / vessel.hull.LPP,
            color="b",
            linestyle="-",
            label="SIM")

    if val_hist is not None:
        n_val = val_hist[:, vessel.val_idx_n]  # n_hist (m)
        e_val = val_hist[:, vessel.val_idx_e]  # e_hist (m)
        if scale == "full-scale":
            axes.plot(
                e_val / vessel.hull.LPP / vessel.SC,
                n_val / vessel.hull.LPP / vessel.SC,
                color="r",
                linestyle="--",
                label="EXP")
        elif scale == "model-scale":
            axes.plot(
                e_val / vessel.hull.LPP,
                n_val / vessel.hull.LPP,
                color="r",
                linestyle="--",
                label="EXP")
        else:
            ValueError(scale, "is not properly defined!")

    axes.set_xlabel(r"$y_0/L_{pp}$ (-)")
    axes.set_ylabel(r"$x_0/L_{pp}$ (-)")
    axes.legend(loc="best")
    axes.axis("equal")
    axes.grid(True)

    fig.tight_layout()


def plot_path_ms_dim(fig, axes, vessel, sim_hist_ms, val_hist=None):
    axes.cla()
    n_hist = sim_hist_ms[:, vessel.hist_idx_n]  # n_hist (m)
    e_hist = sim_hist_ms[:, vessel.hist_idx_e]  # e_hist (m)
    axes.plot(e_hist, n_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        n_val = val_hist[:, vessel.val_idx_n] / vessel.SC  # n_hist (m)
        e_val = val_hist[:, vessel.val_idx_e] / vessel.SC  # e_hist (m)
        axes.plot(e_val, n_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"$y_0$ (m)")
    axes.set_ylabel(r"$x_0$ (m)")
    axes.legend(loc="best")
    axes.axis("equal")
    axes.grid(True)

    fig.tight_layout()


def plot_path_fs_dim(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    n_hist = sim_hist[:, vessel.hist_idx_n]  # n_hist (m)
    e_hist = sim_hist[:, vessel.hist_idx_e]  # e_hist (m)
    axes.plot(e_hist, n_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        n_val = val_hist[:, vessel.val_idx_n]  # n_hist (m)
        e_val = val_hist[:, vessel.val_idx_e]  # e_hist (m)
        axes.plot(e_val, n_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"$y_0$ (m)")
    axes.set_ylabel(r"$x_0$ (m)")
    axes.legend(loc="best")
    axes.axis("equal")
    axes.grid(True)

    fig.tight_layout()


def plot_delta_psi(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
    psi_deg_hist = sim_hist[:, vessel.hist_idx_psi_deg]  # psi_deg_hist (deg)
    delta_deg_hist = sim_hist[:, vessel.hist_idx_delta_deg]  # delta_deg (deg)
    axes.plot(t_hist, psi_deg_hist, color="b", linestyle=":", label="SIM $\psi$")
    axes.plot(t_hist, delta_deg_hist, color="b", linestyle="-", label="SIM $\delta$")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        psi_deg_val = val_hist[:, vessel.val_idx_psi_deg]  # psi_deg (deg)
        delta_deg_val = val_hist[:, vessel.val_idx_delta_deg]  # delta_deg (deg)
        axes.plot(t_val, psi_deg_val, color="r", linestyle="-.", label="EXP $\psi$")
        axes.plot(t_val, delta_deg_val, color="r", linestyle="--", label="EXP $\delta$")

    axes.set_xlabel(r"$Time$ (s)")
    axes.set_ylabel(r"Angle (deg)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_delta(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    if sim_hist is not None:
        t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
        if vessel.n_rudder == 1:
            delta_deg_hist = sim_hist[:, vessel.hist_idx_delta_deg]  # delta_deg (deg)
            axes.plot(t_hist, delta_deg_hist, color="b", linestyle="-", label="SIM Port")
        elif vessel.n_rudder == 2:
            delta_port_deg_hist = sim_hist[:, vessel.hist_idx_delta_port_deg]  # delta_deg (deg)
            axes.plot(t_hist, delta_port_deg_hist, color="b", linestyle="-", label="SIM Port")
            delta_star_deg_hist = sim_hist[:, vessel.hist_idx_delta_star_deg]  # delta_deg (deg)
            axes.plot(t_hist, delta_star_deg_hist, color="b", linestyle="--", label="SIM Star")
        else:
            ValueError("vessel.n_rudder = ", str(vessel.n_rudder), "is not properly defined!")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        if vessel.n_rudder == 1 and vessel.val_idx_delta_deg is not None:
            delta_deg_val = val_hist[:, vessel.val_idx_delta_deg]  # delta_deg (deg)
            axes.plot(t_val, delta_deg_val, color="r", linestyle="--", label="EXP")
        elif vessel.n_rudder == 2 and \
                vessel.val_idx_delta_port_deg is not None and vessel.val_idx_delta_star_deg is not None:
            delta_port_deg_val = val_hist[:, vessel.val_idx_delta_port_deg]  # delta_port_deg (deg)
            axes.plot(t_val, delta_port_deg_val, color="r", linestyle="-", label="EXP Port")
            delta_star_deg_val = val_hist[:, vessel.val_idx_delta_star_deg]  # delta_star_deg (deg)
            axes.plot(t_val, delta_star_deg_val, color="r", linestyle="--", label="EXP Star")
        else:
            ValueError("vessel.n_rudder = ", str(vessel.n_rudder), "is not properly defined!")

    axes.set_xlabel(r"$Time$ (s)")
    axes.set_ylabel(r"$\delta$ (deg)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_psi(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    if sim_hist is not None:
        t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
        psi_deg_hist = sim_hist[:, vessel.hist_idx_psi_deg]  # psi_deg_hist (deg)
        axes.plot(t_hist, psi_deg_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        psi_deg_val = val_hist[:, vessel.val_idx_psi_deg]  # psi_deg (deg)
        axes.plot(t_val, psi_deg_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"$Time$ (s)")
    axes.set_ylabel(r"$\psi$ (deg)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_rps(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    if sim_hist is not None:
        t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
        if vessel.n_propeller == 1:
            rps_hist = sim_hist[:, vessel.hist_idx_rps]  # RPS (Hz)
            axes.plot(t_hist, rps_hist, color="b", linestyle="-", label="SIM")
        elif vessel.n_propeller == 2:
            rps_port_hist = sim_hist[:, vessel.hist_idx_rps_port]  # RPS (Hz)
            axes.plot(t_hist, rps_port_hist, color="b", linestyle="-", label="SIM Port")
            rps_star_hist = sim_hist[:, vessel.hist_idx_rps_port]  # RPS (Hz)
            axes.plot(t_hist, rps_star_hist, color="b", linestyle="--", label="SIM Star")
        else:
            ValueError("vessel.n_propeller = ", str(vessel.n_propeller), "is not properly defined!")
    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        if vessel.n_propeller == 1 and vessel.val_idx_rps is not None:
            rps_val = val_hist[:, vessel.val_idx_rps]  # RPS (Hz)
            axes.plot(t_val, rps_val, color="r", linestyle="--", label="EXP")

        if vessel.n_propeller == 2 and vessel.val_idx_rps_port is not None and vessel.val_idx_rps_star is not None:
            rps_port_val = val_hist[:, vessel.val_idx_rps_port]  # RPS (Hz)
            axes.plot(t_val, rps_port_val, color="r", linestyle="-", label="EXP Port")
            rps_star_val = val_hist[:, vessel.val_idx_rps_star]  # RPS (Hz)
            axes.plot(t_val, rps_star_val, color="r", linestyle="--", label="EXP Star")
        else:
            ValueError("vessel.n_propeller = ", str(vessel.n_propeller), "is not properly defined!")

    axes.set_xlabel(r"Time (s)")
    axes.set_ylabel(r"RPS (Hz)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_r(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
    r_deg_hist = sim_hist[:, vessel.hist_idx_r_deg]  # r_deg (deg/s)
    axes.plot(t_hist, r_deg_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        r_deg_val = val_hist[:, vessel.val_idx_r_deg]  # r_deg (deg/s)
        axes.plot(t_val, r_deg_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"Time (s)")
    axes.set_ylabel(r"$r$ (deg/s)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_u(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
    u_hist = sim_hist[:, vessel.hist_idx_u]  # u (m/s)
    axes.plot(t_hist, u_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        u_val = val_hist[:, vessel.val_idx_u]  # u (m/s)
        axes.plot(t_val, u_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"Time (s)")
    axes.set_ylabel(r"$u$ (m/s)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_v(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
    v_hist = sim_hist[:, vessel.hist_idx_v]  # v (m/s)
    axes.plot(t_hist, v_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        v_val = val_hist[:, vessel.val_idx_v]  # v (m/s)
        axes.plot(t_val, v_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"Time (s)")
    axes.set_ylabel(r"$v$ (m/s)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_beta(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
    u_hist = sim_hist[:, vessel.hist_idx_u]  # u (m/s)
    v_hist = sim_hist[:, vessel.hist_idx_v]  # v (m/s)
    beta_hist = np.arctan(-v_hist / u_hist)

    axes.plot(t_hist, beta_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        u_val = val_hist[:, vessel.val_idx_u]  # u (m/s)
        v_val = val_hist[:, vessel.val_idx_v]  # v (m/s)
        beta_val = np.arctan(-v_val / u_val)

        axes.plot(t_val, beta_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"Time (s)")
    axes.set_ylabel(r"$\beta$ (deg)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_U(fig, axes, vessel, sim_hist=None, val_hist=None):
    axes.cla()
    t_hist = sim_hist[:, vessel.hist_idx_t]  # Time (s)
    u_hist = sim_hist[:, vessel.hist_idx_u]  # u (m/s)
    v_hist = sim_hist[:, vessel.hist_idx_v]  # v (m/s)
    U_hist = np.hypot(u_hist, v_hist)  # U (m/s)

    axes.plot(t_hist, U_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        t_val = val_hist[:, vessel.val_idx_t]  # Time (s)
        u_val = val_hist[:, vessel.val_idx_u]  # u (m/s)
        v_val = val_hist[:, vessel.val_idx_v]  # v (m/s)
        U_val = np.hypot(u_val, v_val)  # U (m/s)

        axes.plot(t_val, U_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"Time (s)")
    axes.set_ylabel(r"$U$ (m/s)")
    axes.legend(loc="best")
    axes.grid(True)

    fig.tight_layout()


def plot_track(fig, axes, vessel, sim_hist, path_cmd, val_hist=None):
    axes.cla()
    n_hist = sim_hist[:, vessel.hist_idx_n]  # n_hist (m)
    e_hist = sim_hist[:, vessel.hist_idx_e]  # e_hist (m)

    axes.plot(path_cmd.coord[:, 1], path_cmd.coord[:, 0], color="r", linestyle="", marker="o", markersize=5,
              label="REF")
    axes.plot(e_hist, n_hist, color="b", linestyle="-", label="SIM")

    if val_hist is not None:
        n_val = val_hist[:, vessel.val_idx_n]  # n_hist (m)
        e_val = val_hist[:, vessel.val_idx_e]  # e_hist (m)
        axes.plot(e_val, n_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"$y_0$ (m)")
    axes.set_ylabel(r"$x_0$ (m)")
    axes.legend(loc="best")
    axes.axis("equal")
    axes.grid(True)

    fig.tight_layout()


def plot_avoidance(fig, axes, vessel, sim_hist, start_point, end_point, obstacles, path_plan, path_cmd, val_hist=None):
    axes.cla()  # axes.cla()函数用于清除当前的坐标轴
    n_hist = sim_hist[:, vessel.hist_idx_n]  # n_hist (m)
    e_hist = sim_hist[:, vessel.hist_idx_e]  # e_hist (m)

    map_tank_x = [69.94703527893999, 86.13690327184614, 28.40204810847227, 9.57336173850015, 69.94703527893999]
    map_tank_y = [5.512732278904286, 83.15994926147745, 96.65465848643316, 18.8830221993, 5.512732278904286]
    axes.plot(map_tank_x, map_tank_y, color='k', linestyle="-", label="Bank")
    axes.plot(start_point[1], start_point[0], color="y", linestyle="None", marker="o", markersize=5, label="Start path_point")
    axes.plot(end_point[1], end_point[0], color="m", linestyle="None", marker="o", markersize=5, label="End path_point")
    axes.plot(obstacles[:, 1], obstacles[:, 0], color="r", linestyle="None", marker="s", markersize=5, label="Obstacles")
    axes.plot(path_plan.coord[:, 1], path_plan.coord[:, 0], color="g", linestyle="--", label="Planned path")
    axes.plot(path_cmd.coord[:, 1], path_cmd.coord[:, 0], color="y", linestyle="", marker="*", markersize=5, label="Commanded path")
    axes.plot(e_hist, n_hist, color="b", linestyle="-", label="Actual path")
    # axes.plot.pause(0.1)

    if val_hist is not None:
        n_val = val_hist[:, vessel.val_idx_n]  # n_hist (m)
        e_val = val_hist[:, vessel.val_idx_e]  # e_hist (m)
        axes.plot(e_val, n_val, color="r", linestyle="--", label="EXP")

    axes.set_xlabel(r"$y_0$ (m)")
    axes.set_ylabel(r"$x_0$ (m)")
    axes.legend(loc="best")
    axes.axis("equal")
    axes.grid(True)

    fig.tight_layout()


def plot_potential_map(fig, axes, potential_map, field_plan, val_hist=None):
    axes.cla()
    v_min = np.min(potential_map)
    v_max = np.max(potential_map) / 10

    axes.pcolor(np.array(potential_map), vmin=v_min, vmax=v_max, cmap="GnBu")

    axes.plot(field_plan[:, 1], field_plan[:, 0], linestyle="--", color="r", label="Path plan")
    axes.plot(
        field_plan[0, 1], field_plan[0, 0], color="y", linestyle="None", marker="o", markersize=5,
        label="Start path_point")
    axes.plot(
        field_plan[-1, 1], field_plan[-1, 0], color="m", linestyle="None", marker="o", markersize=5,
        label="End path_point")

    axes.set_xlabel(r"$y_0$ (m)")
    axes.set_ylabel(r"$x_0$ (m)")
    axes.legend(loc="best")
    axes.axis("equal")
    axes.grid(True)

    fig.tight_layout()


def plot_all_info(fig, axes, vessel, sim_hist=None, val_hist=None, scale="full-scale"):
    ax_rps = axes[0, 0]
    ax_delta = axes[0, 1]
    ax_path_non = axes[0, 2]
    ax_U = axes[1, 0]
    ax_u = axes[1, 1]
    ax_v = axes[1, 2]
    ax_psi = axes[2, 0]
    ax_r = axes[2, 1]
    ax_beta = axes[2, 2]

    plot_rps(fig, ax_rps, vessel, sim_hist, val_hist)
    plot_delta(fig, ax_delta, vessel, sim_hist, val_hist)
    plot_path_fs_non(fig, ax_path_non, vessel, sim_hist, val_hist, scale)
    plot_U(fig, ax_U, vessel, sim_hist, val_hist)
    plot_u(fig, ax_u, vessel, sim_hist, val_hist)
    plot_v(fig, ax_v, vessel, sim_hist, val_hist)
    plot_psi(fig, ax_psi, vessel, sim_hist, val_hist)
    plot_r(fig, ax_r, vessel, sim_hist, val_hist)
    plot_beta(fig, ax_beta, vessel, sim_hist, val_hist)
