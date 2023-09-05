import numpy as np
import math
import AISDataGet


def gain_k(v):
    k1 = 10 ** (0.3591 * math.log10(v) + 0.0952)
    k2 = 10 ** (0.5441 * math.log10(v) - 0.0795)
    return k1, k2


def quaternion(k1, k2, l):  # 计算船舶四个方向上船舶域的半径
    r_aft = (1 + 0.67 * math.sqrt(k1 ** 2 + (k2 / 2) ** 2)) * l * 3
    return r_aft


def cheng(v1, v2):
        g = np.dot(v1, v2)  # 向量之间的点乘函数
        return g


def angle_of_vector(v1, v2):   # 航向与船舶相对位置夹角计算函数
        vector_prod = v1[0] * v2[0] + v1[1] * v2[1]
        length_prod = math.sqrt(pow(v1[0], 2) + pow(v1[1], 2)) * math.sqrt(pow(v2[0], 2) + pow(v2[1], 2))
        cos = vector_prod/length_prod
        angle = math.acos(cos)
        ang = angle * 180/np.pi
        return ang


def cr(v1, v2):  # 判断向量叉乘值的正负,以此来判断夹角正负
        z_theta = np.cross(v1, v2)
        return z_theta


def WaypointGet(Lat, Lon, COG, SOG, DCPA, TCPA, DCPA_lim, TCPA_lim, x, y, wpx, wpy):
    L = 160 / 1852  # 船长
    r = 0.5
    k = 2  # 船舶域计算参数
    n = len(x)  # 总的船舶数量
    x_tra = []
    y_tra = []  # 将距离单位转换成海里
    '''zijian---------------------添加船舶标签------------------------------'''  # FIXME Zhang Zijian
    ship_num = ['TS1', 'TS2', 'TS3', 'TS4', 'TS5', 'TS6',
                'TS7', 'TS8', 'TS9', 'TS10', 'TS11', 'TS12', 'TS13', 'TS14', 'TS15', 'TS16', 'TS17', 'TS18', 'TS19',
                'TS20',  'TS21', 'OS']  # 船舶标签库
    waypoint_num = ['OT1.waypoint', 'OT2.waypoint', 'OT3.waypoint', 'OT4.waypoint',
                    'OT5.waypoint', 'OT6.waypoint', 'OT7.waypoint', 'OT8.waypoint',
                    'OT9.waypoint', 'OT10.waypoint', 'OT11.waypoint','OT12.waypoint',
                    'OT13.waypoint', 'OT14.waypoint', 'OT15.waypoint', 'OT16.waypoint',
                    'OT17.waypoint', 'OT18.waypoint', 'OT19.waypoint', 'OT20.waypoint', 'OT21.waypoint', 'OS.waypoint']  # 船舶路径点标签库
    label_ship = []  # 出现在本船一定范围内的可以识别的目标船的标签（包括本船）
    for i in range(len(wpx)):
        wpx[i] = wpx[i] / 1852
        wpy[i] = wpy[i] / 1852
    for i in range(0, n - 1):
        x_tra.append(x[i] / 1852)
        y_tra.append(y[i] / 1852)
        label_ship.append(ship_num[i])
    x_tra.append(x[n - 1] / 1852)
    y_tra.append(y[n - 1] / 1852)
    label_ship.append(ship_num[len(ship_num)-1])
    label_waypoint = []  # 出现在本船一定范围内的可以识别的目标船的标签（包括本船）
    x_area = []  # 出现在本船一定范围内的可以识别的目标船的x坐标
    y_area = []  # 出现在本船一定范围内的可以识别的目标船的y坐标
    COG_area = []  # 出现在本船一定范围内的可以识别的目标船的航向（包括本船）
    SOG_area = []  # 出现在本船一定范围内的可以识别的目标船的航速（包括本船）
    DCPA_area = []
    TCPA_area = []
    for i in range(0, n - 1):  # 将符合条件的路径点标签加到列表中
        if math.sqrt((x_tra[i] - x_tra[n - 1]) ** 2 + (y_tra[i] - y_tra[n - 1]) ** 2) < 5:
            x_area.append(x_tra[i])
            y_area.append(y_tra[i])
            COG_area.append(COG[i])
            SOG_area.append(SOG[i])  # 将符条件的船舶信息添加到新的列表
            DCPA_area.append(DCPA[i])
            TCPA_area.append(TCPA[i])
            label_waypoint.append(waypoint_num[i])
    x_area.append(x_tra[n-1])
    y_area.append(y_tra[n-1])
    COG_area.append(COG[n-1])
    SOG_area.append(SOG[n-1])
    label_waypoint.append(waypoint_num[len(waypoint_num)-1])
    m = len(x_area)  # 获取在一定范围内的船舶数量
    waypoint = ['']
    CAL = []
    CPA = []
    THETA1 = []  # 本船航向与目标船与本船相对位置的夹角
    THETA2 = []  # 本船航向与目标船航向的夹角
    x_risk = []
    y_risk = []
    warn = 0
    WpCog = []
    index_value = 0
    vision_value = []
    pos_vision = [x_area[m - 1], y_area[m - 1]]
    for i in range(0, m - 1):
        kad, kdt = gain_k(SOG_area[i])
        r_aft = quaternion(kad, kdt, L)
        # 将速度转化为矢量
        VT = np.array([SOG_area[i] * math.sin(math.radians(COG_area[i])),
                       SOG_area[i] * math.cos(math.radians(COG_area[i])), 0])
        VOS = np.array([SOG_area[m - 1] * math.sin(math.radians(COG_area[m - 1])),
                        SOG_area[m - 1] * math.cos(math.radians(COG_area[m - 1])), 0])
        posOT = np.array([x_area[i] - x_area[m - 1], y_area[i] - y_area[m - 1], 0])  # 本船与目标船相对位置向量
        t1 = angle_of_vector(VOS, posOT)
        t2 = angle_of_vector(VOS, VT)
        z1 = cr(VOS, posOT)[2]
        z2 = cr(VOS, VT)[2]
        if z1 > 0:
            theta1 = -t1
        elif z1 == 0:
            theta1 = 180
        else:
            theta1 = t1
        if z2 > 0:
            theta2 = -t2
        elif z2 == 0:
            theta2 = 180
        else:
            theta2 = t2
        #  以上为判断场景的条件，下面为相应场景下路径点的计算
        #  1、追越场景
        if (-5 <= theta2 <= 5) and (SOG_area[m-1] > SOG_area[i]) and (-90 < theta1 < 90):
            vision_value.append(0)
            cpa = ([x_area[i] + (SOG_area[i] * math.sin(math.radians(COG_area[i])) * (TCPA_area[i] / 3600)),
                    y_area[i] + (SOG_area[i] * math.cos(math.radians(COG_area[i])) * (TCPA_area[i] / 3600)), 0])
            CPA.append(cpa)
            CAL.append(3)
        #  2、对遇场景
        elif (-180 <= theta2 <= -175 or 175 <= theta2 <= 180) and (-90 < theta1 < 90):
            vision_value.append(0)
            cpa = ([x_area[i] + (SOG_area[i] * math.sin(math.radians(COG_area[i])) * (TCPA_area[i] / 3600)),
                    y_area[i] + (SOG_area[i] * math.cos(math.radians(COG_area[i])) * (TCPA_area[i] / 3600)), 0])
            CPA.append(cpa)
            CAL.append(3)
        #  3、交叉会遇场景
        else:
            vision_value.append(1)
            if 0 < DCPA_area[i] < DCPA_lim and 0 < TCPA_area[i] < TCPA_lim:
                x_risk.append(x_tra[i])
                y_risk.append(y_tra[i])
                if 112.5 > theta1 > -5:
                    cal = 1  # 本船执行让路操作，从目标船船尾经过
                else:
                    cal = 0  # 本船执行直航操作，从目标船船头经过
            else:
                cal = ' '
            # 计算会遇点位置cpa
            cpa = ([x_area[i] + (SOG_area[i] * math.sin(math.radians(COG_area[i])) * (TCPA_area[i]/3600)),
                    y_area[i] + (SOG_area[i] * math.cos(math.radians(COG_area[i])) * (TCPA_area[i]/3600)), 0])
            CAL.append(cal)
            CPA.append(cpa)
        THETA2.append(theta2)
        THETA1.append(theta1)
    D_min = 2 ** 20
    h = 0
    for i in range(0, m - 1):  # 判断最小会遇距离的路径点
        #  1、计算追越场景路径点
        if (-5 <= THETA2[i] <= 5) and (SOG_area[m-1] > SOG_area[i]) and (-90 < THETA1[i] < 90):
            if DCPA_area[i] == 0:
                D_min = DCPA_area[i]
                h = i
                waypoint = [1*math.sin(math.radians(COG_area[m-1]+30)),
                            1*math.cos(math.radians(COG_area[m-1]+30))]
            elif 0 < DCPA_area[i] < DCPA_lim and DCPA_area[i] < D_min and 0 < TCPA_area[i] < TCPA_lim:
                D_min = DCPA_area[i]
                h = i

                if THETA1[h] < 0:
                    waypoint = [(1-2*DCPA_area[h]) * math.sin(math.radians(COG_area[m - 1]+30)),  # FIXME Zhang Zijian
                                (1-2*DCPA_area[h]) * math.cos(math.radians(COG_area[m - 1]+30))]
                elif THETA1[h] > 0:
                    waypoint = [(1+2 * DCPA_area[h]) * math.sin(math.radians(COG_area[m - 1]+30)),  # FIXME Zhang Zijian
                                (1+2 * DCPA_area[h]) * math.cos(math.radians(COG_area[m - 1]+30))]
        #  2、计算对遇场景路径点
        elif (-180 <= THETA2[i] <= -175 or 175 <= THETA2[i] <= 180) and (-90 < THETA1[i] < 90):
            if DCPA_area[i] == 0:
                D_min = DCPA_area[i]
                h = i
                waypoint = [1 * math.sin(math.radians(COG_area[m - 1] + 30)),
                            1 * math.cos(math.radians(COG_area[m - 1] + 30))]
            elif 0 < DCPA_area[i] < DCPA_lim and DCPA_area[i] < D_min and 0 < TCPA_area[i] < TCPA_lim:
                D_min = DCPA_area[i]
                h = i
                if THETA1[h] < 0:
                    waypoint = [(1 - 2 * DCPA_area[h]) * math.sin(math.radians(COG_area[m - 1] + 30)),
                                (1 - 2 * DCPA_area[h]) * math.cos(math.radians(COG_area[m - 1] + 30))]
                elif THETA1[h] > 0:
                    waypoint = [(1 + 2 * DCPA_area[h]) * math.sin(math.radians(COG_area[m - 1] + 30)),
                                (1 + 2 * DCPA_area[h]) * math.cos(math.radians(COG_area[m - 1] + 30))]
        #  3、计算交叉场景路径点
        else:
            if CAL[i] == 1 and -180 < THETA2[i] < 0 and 0 <= THETA1[i] <= 112.5 and 0 < TCPA_area[i] < TCPA_lim:
                if DCPA_area[i] < D_min:
                   D_min = DCPA_area[i]
                   h = i
                   waypoint = [CPA[h][0] - r_aft * math.sin(math.radians(COG_area[h])),
                               CPA[h][1] - r_aft * math.cos(math.radians(COG_area[h]))]
                   pos_vision[0] = CPA[h][0]
                   pos_vision[1] = CPA[h][1]
                   if 67.5 < THETA1[h] < 112.5:
                       waypoint = [math.sin(math.radians(COG_area[m - 1])), math.cos(math.radians(COG_area[m - 1]))]
                       warn = 1
                       pos_vision[0] = x_area[m - 1]
                       pos_vision[1] = y_area[m - 1]
            elif CAL[i] == 1 and 0 < THETA2[i] < 180 and -5 <= THETA1[i] <= 0 and 0 < TCPA_area[i] < TCPA_lim:
                if DCPA_area[i] < D_min:
                   D_min = DCPA_area[i]
                   h = i
                   waypoint = [CPA[h][0] - r_aft * math.sin(math.radians(COG_area[h])),
                               CPA[h][1] - r_aft * math.cos(math.radians(COG_area[h]))]
                   pos_vision[0] = CPA[h][0]
                   pos_vision[1] = CPA[h][1]
        #  4、无风险路径点
    if waypoint[0] == '':
        h = m - 1
        if math.sqrt((wpx[index_value] - x_tra[n - 1]) ** 2 + (wpy[index_value] - y_tra[n - 1]) ** 2) < 1:
            index_value = index_value + 1
        else:
            if index_value <= len(wpx) - 1:
                waypoint = [wpx[index_value], wpy[index_value]]
                a1 = np.array([0, 2, 0])
                a2 = np.array([wpx[index_value] - x_tra[n - 1], wpy[index_value] - y_tra[n - 1], 0])
                t3 = angle_of_vector(a1, a2)
                z3 = cr(a1, a2)[2]
                if z3 > 0:
                    WpCog = -t3
                elif z3 == 0:
                    WpCog = 180
                else:
                    WpCog = t3

    return waypoint, x_tra, y_tra, n, m, h, label_ship, label_waypoint, warn, WpCog, index_value, CPA, pos_vision, COG_area, vision_value


if __name__ == "__main__":
    Lat, Lon, COG, SOG, DCPA, TCPA, DCPA_lim, TCPA_lim, lat_wp, lon_wp = AISDataGet.RedisDataGet()
    position_all_x, position_all_y, wpx, wpy = AISDataGet.LatLon2Cartesian(Lat, Lon, lat_wp, lon_wp)
    waypoint, x_tra, y_tra, n, m, h, label_ship, label_waypoint, warn, WpCog, index_value, CPA, pos_vision, COG_area, vision_value = WaypointGet(Lat, Lon, COG, SOG, DCPA, TCPA, DCPA_lim, TCPA_lim, position_all_x, position_all_y, wpx, wpy)
    print(waypoint, x_tra, y_tra, n, m, h, label_ship, label_waypoint, warn, WpCog, index_value, CPA, pos_vision)
    print(position_all_x, position_all_y, wpx, wpy)
    print(Lat, Lon, position_all_x, position_all_y, DCPA, TCPA, len(CPA), vision_value)
