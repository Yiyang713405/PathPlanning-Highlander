import numpy as np
import math
import matplotlib.pyplot as plt
import redis
import json
import time

import AISDataGet
import shipmodle
import WaypointGet
import PotentialFieldPlanning
import a_star
import bezier_path
import bspline_path
import PointDelete
import csv

from service import tanker_KVLCC2_MARIN_L7
from service import ip_transform
from service import ip_solver
from service import ip_help
from service.ip_field import OpenSea


def maneuver_prediction(speed, heading, delta):
    motion_solver = ip_solver.MotionSolver(t_start=0.0, t_step=0.1, t_end=10.0)

    ship_param = tanker_KVLCC2_MARIN_L7.Kvlcc2MarinL7()
    field_param = OpenSea

    # ship_param.u = ship_param.U_serv  # (m/s)
    ship_param.u = speed
    ship_param.propeller.rps = ship_param.propeller.rps_serv  # (Hz)
    ship_param.rudder.delta_deg = heading  # (deg)
    ship_param.rudder.trans_delta_deg2rad()

    ship_param.propeller.rps_cmd = ship_param.propeller.rps_serv  # (Hz)
    ship_param.rudder.delta_cmd_deg = delta  # (deg)

    # Simulation
    print("\n" + "*" * 15 + " Start " + "*" * 15)
    with ip_help.IpTimer():
        sim_hist_ms_rad = motion_solver.solve(field_param, ship_param)
        sim_hist_ms_deg = ip_transform.hist_rad2deg(ship_param, sim_hist_ms_rad)
    print("*" * 16 + " End " + "*" * 16 + "\n")

    sim_hist_fs_deg = ip_transform.hist_m2f(ship_param, sim_hist_ms_deg)

    return sim_hist_fs_deg


AISDataGet.RedisDataWrite()
Mile2m = 1852

plt.ion()
fig, axes = plt.subplots()
plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
start_point = [0, 0]
start_x = start_point[0]
start_y = start_point[1]
target1_lat = []   # FIXME Zhang Zijian
target1_lon = []   # 添加所需导出数据的列表
target2_lat = []
target2_lon = []
target3_lat = []
target3_lon = []
target4_lat = []
target4_lon = []
target5_lat = []
target5_lon = []
target6_lat = []
target6_lon = []
ownship_lat = []
ownship_lon = []
target1_cog = []
target2_cog = []
target3_cog = []
target4_cog = []
target5_cog = []
target6_cog = []
ownship_cog = []
while True:
    axes.set_aspect('equal')
    Lat, Lon, COG, SOG, DCPA, TCPA, CPA_lim, TCPA_lim, lat_wp, lon_wp = AISDataGet.RedisDataGet()
    position_all_x, position_all_y, wpx, wpy = AISDataGet.LatLon2Cartesian(Lat, Lon, lat_wp, lon_wp)
    waypoint, x_tra, y_tra, n, m, h, label_ship, label_waypoint, warn, WpCog, index_value, CPA, pos_vision, COG_area, vision_value = WaypointGet.WaypointGet(
        Lat, Lon, COG, SOG, DCPA, TCPA, CPA_lim, TCPA_lim, position_all_x, position_all_y, wpx, wpy)
    icon = shipmodle.shipmodle(COG, x_tra, y_tra, n, CPA, h, m, COG_area, vision_value)

    obstacles_x = np.delete(x_tra, -1)
    obstacles_y = np.delete(y_tra, -1)

    end_point = waypoint
    goal_x = end_point[0]
    goal_y = end_point[1]

    '''---------------------APF---start------------------------------'''
    # grid_size = 0.1  # Resolution of the potential rho (m)
    # area_radius = 0.5  # Potential area radius (m)
    # rx_apf, ry_apf = PotentialFieldPlanning.potential_field_planning(start_x, start_y, goal_x, goal_y, obstacles_x,
    #                                                          obstacles_y, grid_size, area_radius)
    '''---------------------APF---end---------------------------------'''
    '''---------------------AStar---start-----------------------------'''
    # ASPlanner = a_star.AStarPlanner(obstacles_x, obstacles_y, grid_size, area_radius)
    # rx_astar, ry_astar = ASPlanner.planning(start_x, start_y, goal_x, goal_y)
    '''---------------------AStar---end-----------------------------'''
    '''---------------------bezier_path---start------------------------------'''
    angle_trans = (360 - COG[-1]) % 360
    angle_trans = (angle_trans + 90) % 360  # 将顺时针纵轴向上为0°的角度数据转化为逆时针横轴向右为0°的角度数据
    start_yaw = np.radians(angle_trans)
    end_yaw = start_yaw
    path, control_points = bezier_path.calc_4points_bezier_path(start_x, start_y, start_yaw, goal_x, goal_y, end_yaw, offset=3)
    rx_bezier = path.T[0]
    ry_bezier = path.T[1]
    '''---------------------bezier_path---start------------------------------'''
    # print information
    rx = rx_bezier
    ry = ry_bezier
    lenth = len(rx)
    print(len(rx))
    rx_for_redis = rx_bezier * Mile2m
    ry_for_redis = ry_bezier * Mile2m
    print("rx:", rx, "\n", "ry:", ry, sep='')
    print(COG, Lon)
    '''zijian---------------------读取船舶的AIS数据信息并写入csv文件导出------------------------------'''   # FIXME Zhang Zijian
    # target1_lat.append(Lat[0])
    # target1_lon.append(Lon[0])
    # target2_lat.append(Lat[1])
    # target2_lon.append(Lon[1])
    # target3_lat.append(Lat[2])
    # target3_lon.append(Lon[2])
    # target4_lat.append(Lat[3])
    # target4_lon.append(Lon[3])
    # target5_lat.append(Lat[4])
    # target5_lon.append(Lon[4])
    # target6_lat.append(Lat[5])
    # target6_lon.append(Lon[5])
    # ownship_lat.append(Lat[6])
    # ownship_lon.append(Lon[6])
    # target1_cog.append(COG[0])
    # target2_cog.append(COG[1])
    # target3_cog.append(COG[2])
    # target4_cog.append(COG[3])
    # target5_cog.append(COG[4])
    # target6_cog.append(COG[5])
    # ownship_cog.append(COG[6])
    #
    # with open('E:/TugData/test.csv', 'w', newline='') as file:
    #     writer = csv.writer(file)
    #     writer.writerow(['target1_lat', 'target1_lon', 'target2_lat', 'target2_lon', 'target3_lat', 'target3_lon',
    #                      'target4_lat', 'target4_lon', 'target5_lat', 'target5_lon', 'target6_lat', 'target6_lon',
    #                      'ownship_lat', 'ownship_lon', 'target1_cog', 'target2_cog',
    #                      'target3_cog', 'target4_cog', 'target5_cog', 'target6_cog',
    #                      'ownship_cog'])  # 写入表头
    #     for i in range(len(target1_lat)):
    #         writer.writerow(
    #             [target1_lat[i], target1_lon[i], target2_lat[i], target2_lon[i], target3_lat[i], target3_lon[i],
    #              target4_lat[i], target4_lon[i], target5_lat[i], target5_lon[i], target6_lat[i], target6_lon[i],
    #              ownship_lat[i], ownship_lon[i], target1_cog[i], target2_cog[i], target3_cog[i], target4_cog[i],
    #              target5_cog[i], target6_cog[i], ownship_cog[i]])  # 写入每行数据
    # target1_lat.append(Lat[0])
    # target1_lon.append(Lon[0])
    # ownship_lat.append(Lat[1])
    # ownship_lon.append(Lon[1])
    # target1_cog.append(COG[0])
    # ownship_cog.append(COG[1])

    # with open('C:/Users/Administrator/Desktop/WHUT_test_0411/test.csv', 'w', newline='') as file:
    #     writer = csv.writer(file)
    #     writer.writerow(['target1_lat', 'target1_lon', 'target2_lat', 'target2_lon', 'target3_lat', 'target3_lon',
    #                      'target4_lat', 'target4_lon', 'target5_lat', 'target5_lon', 'target6_lat', 'target6_lon',
    #                      'ownship_lat', 'ownship_lon', 'target1_cog', 'target2_cog',
    #                      'target3_cog', 'target4_cog', 'target5_cog', 'target6_cog',
    #                      'ownship_cog'])  # 写入表头
    #     for i in range(len(target1_lat)):
    #         writer.writerow([target1_lat[i], target1_lon[i], ownship_lat[i], ownship_lon[i],
    #                          target1_cog[i], ownship_cog[i]])  # 写入每行数据
    '''zijian---------------------end------------------------------'''
    rxfinal, ryfinal = PointDelete.pd(rx, ry)
    Theta_heading_rad = np.arctan((rxfinal[1] - rxfinal[0]) / (ryfinal[1] - ryfinal[0]))
    Theta_heading_deg = np.degrees(Theta_heading_rad)
    # print('Recommendation for heading =', Theta_heading_deg)
    if Theta_heading_deg < 0:
        Theta_heading_deg = Theta_heading_deg + 360  # FIXME Zhang Zijian
    pool = redis.ConnectionPool(host='127.0.0.1', port=6379, db=0)
    # pool = redis.ConnectionPool(host='169.254.197.145', port=6379, db=0)
    r = redis.StrictRedis(connection_pool=pool)
    data = {"OSExtraCollisionAvoidancePara": {"AimCourse": Theta_heading_deg,
                                              "CrsChange": Theta_heading_deg,
                                              "RecoverCrs": Theta_heading_deg},
            "ZAvoidingRoute": []}
    for i in range(len(rx_for_redis)):
        item = {
            "posX": rx_for_redis[i],
            "posY": ry_for_redis[i]
        }
        data["ZAvoidingRoute"].append(item)

    json_data = json.dumps(data)
    r.set('Common.Command.Avoiding', json_data)
    # 操纵性预测
    self_speed = 15 * 0.5144  # 15节乘以节与m/s之间的转化系数
    heading = COG[-1]
    print(heading)
    delta_cmd = Theta_heading_deg
    sim_hist_fs_deg = maneuver_prediction(self_speed, heading, delta_cmd)
    y_pred = sim_hist_fs_deg[:, 1] / Mile2m
    x_pred = sim_hist_fs_deg[:, 3] / Mile2m
    psi_pred = sim_hist_fs_deg[:, 5]
    start_x = sim_hist_fs_deg[1, 1] / Mile2m
    start_y = sim_hist_fs_deg[1, 3] / Mile2m
    start_yaw = sim_hist_fs_deg[1, 5]
    COG[-1] = start_yaw
    icon = shipmodle.shipmodle(COG, x_tra, y_tra, n, CPA, h, m, COG_area, vision_value)
    axes.clear()
    for i in range(0, n):  # 画出船舶的位置
        axes.scatter(x_tra[i], y_tra[i], marker=icon[i], color='orange', s=5, edgecolors="black")
        axes.text(x_tra[i], y_tra[i] - 0.4, label_ship[i], ha="center", family='sans-serif', size=8)
    axes.scatter(pos_vision[0], pos_vision[1], marker=icon[len(icon) - 1], color='black', s=5, edgecolors="black")
    axes.text(pos_vision[0], pos_vision[1] - 0.4, 'VisionShip', ha="center", family='sans-serif', size=8)
    axes.scatter(waypoint[0], waypoint[1], s=50, marker='.', color='r')
    for i in range(len(wpx)):
        axes.scatter(wpx[i], wpy[i], marker='^', color='red', s=50, edgecolors="red")
        axes.text(wpx[i], wpy[i] - 0.4, 'Towing Point', ha="center", family='sans-serif', size=8)
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl2 = [1000 / Mile2m * math.cos(np.deg2rad(d)) for d in deg]
    yl2 = [1000 / Mile2m * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl2, yl2, 'r')
    axes.plot(rx, ry, color="b", linestyle="-", label="Planned Path")
    axes.plot(x_pred, y_pred, color="g", linestyle="--", label="Motion Prediction")
    axes.scatter(rxfinal, ryfinal, color="g", s=1, marker='.', label="Planned Point")
    axes.text(waypoint[0], waypoint[1], label_waypoint[h], ha="left", family='sans-serif', size=8)
    axes.grid(True)
    axes.grid(color='black', linestyle='--', linewidth=0.3, alpha=0.3)
    axes.set_xlabel('x(n mile)')
    axes.set_ylabel('y(n mile)')
    axes.legend(loc="upper left")
    axes.grid(True)
    # 显示图形并暂停1秒
    waypoint.clear()
    x_tra.clear()
    y_tra.clear()
    plt.pause(1)
    time.sleep(1)  # 暂停 1秒
    axes.clear()  # 清除之前的图形




