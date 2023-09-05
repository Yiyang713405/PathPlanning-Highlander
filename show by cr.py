import matplotlib.pyplot as plt
import numpy as np
import AISDataGet
import time
import WaypointGet
show_animation = True
from matplotlib.patches import Circle
# from RRT_planning import RRT
import PointDelete


# 交互模式设置
plt.ion()

while True:
    # 创建图形
    fig, ax = plt.subplots()
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    while True:
        ax.set_aspect('equal')
        Lat, Lon, COG, SOG, DCPA, TCPA, CPA_lim, TCPA_lim, lat_wp, lon_wp = AISDataGet.RedisDataGet()  # 读取数据
        position_all_x, position_all_y, wpx, wpy = AISDataGet.LatLon2Cartesian(Lat, Lon, lat_wp, lon_wp)
        waypoint, x_tra, y_tra, n, m, h, label_ship, label_waypoint, warn, WpCog, index_value =WaypointGet.WaypointGet(Lat, Lon, COG, SOG, DCPA, TCPA, CPA_lim, TCPA_lim, position_all_x, position_all_y, wpx, wpy)
        start = [x_tra[-1], y_tra[-1]]
        goal = waypoint
        xmin = min(x_tra)
        xmax = max(x_tra)
        ymin = min(y_tra)
        ymax = min(y_tra)
        rand_area = [x_tra[-1] - 10, x_tra[-1] + 10, y_tra[-1] - 7, y_tra[-1] + 7]
        obstacleList = [[x_tra[h], y_tra[h], 0.1]]  # [x, y, radius]
        # print(x_tra)
        # print(COG)

        # 定义6组位置和航向角的数据
        position_and_heading = [
            ([x_tra[0], y_tra[0]], np.radians(COG[0])),
            ([x_tra[1], y_tra[1]], np.radians(COG[1])),
            ([x_tra[2], y_tra[2]], np.radians(COG[2])),
            ([x_tra[3], y_tra[3]], np.radians(COG[3])),
            ([x_tra[4], y_tra[4]], np.radians(COG[4])),
            ([x_tra[5], y_tra[5]], np.radians(COG[5])),
            ([x_tra[6], y_tra[6]], np.radians(COG[6]))
        ]
        """设置坐标系"""
        ax.set_xlim(x_tra[-1] - 8, x_tra[-1] + 8)
        ax.set_ylim(y_tra[-1] - 6, y_tra[-1] + 6)
        # ax.set_xlim(-10, 10)
        # ax.set_ylim(-7, 7)
        # ========================plan star======================
        # Set Initial parameters
        # rrt = RRT(start=start, goal=goal, rand_area = rand_area, obstacle_list=obstacleList, robot_radius=0.8)
        # path = rrt.planning(animation=show_animation)
        # print(path)
        start_x = start_point[0]
        start_y = start_point[1]
        goal_x = end_point[0]
        goal_y = end_point[1]
        grid_size = 0.1  # Resolution of the potential rho (m)
        area_radius = 0.5  # Potential area radius (m)
        rx, ry = PotentialFieldPlanning.potential_field_planning(start_x, start_y, goal_x, goal_y, obstacles_x,
                                                                 obstacles_y, grid_size, area_radius)
        # end_x = [init_x[0] for init_x in path]      #将path拆为x，y
        # end_y = [init_y[1] for init_y in path]
        rxfinal, ryfinal = PointDelete.pd(end_x, end_y)
        # ax.plot(x_tra[-1], y_tra[-1], 'ro')
        ax.scatter(waypoint[0], waypoint[1], s=15, marker='o', c='green')
        #-----the end path----------
        ax.plot(rxfinal, ryfinal,linestyle='--', color='green')     #画路径
        # ========================plan end======================
        #--------------drawship--------------
        for i in range(0, 7):
            # x和y是位置列表，theta是航向角
            (ship_x, ship_y), theta = position_and_heading[i]
            # theta = np.deg2rad(angle)
            u = np.cos(theta)
            v = np.sin(theta)
            # 绘制箭头
            plt.quiver(ship_x, ship_y, u, v, width=0.01, scale=25, angles='xy', headlength=1, headwidth=1.5,color=(0.88, 0.49, 0.32))
            # # 绘制点
            # ax.scatter(ship_x, ship_y, alpha=1,marker='o')
            # 绘制搜索半径
            circle = Circle((x_tra[-1], y_tra[-1]), 5, color='red', fill=False)
            ax.add_patch(circle)
        # # 设置图形标题和标签
        ax.set_title('Position and Heading')
        ax.set_xlabel('X position/nm')
        ax.set_ylabel('Y position/nm')

        # 显示图形并暂停1秒
        waypoint.clear()
        x_tra.clear()
        y_tra.clear()
        # plt.draw()
        plt.pause(1)
        time.sleep(1)       # 暂停 1秒
        ax.clear()          # 清除之前的图形