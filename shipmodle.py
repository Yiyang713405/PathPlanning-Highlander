import matplotlib.pyplot as plt
import matplotlib.path as mpath
from matplotlib.markers import MarkerStyle
from matplotlib import transforms
import AISDataGet
import WaypointGet


def shipmodle(COG, x_tra, y_tra, n, CPA, h, m, COG_area, vision_value):
    icon = []
    # 添加一个路径path，路径的详细解释后面会讲到，相比于简单的patch，稍显复杂

    class UnsizedMarker(MarkerStyle):
        def _set_custom_marker(self, path):
            self._transform = transforms.IdentityTransform()
            self._path = path

    for i in range(0, n):
        Path = mpath.Path
        ship = Path([[x_tra[i] - 1, y_tra[i]], [x_tra[i] - 1, y_tra[i] + 3], [x_tra[i], y_tra[i] + 6],
                     [x_tra[i] + 1, y_tra[i] + 3], [x_tra[i] + 1, y_tra[i]], [x_tra[i], y_tra[i]]],
                    [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY])
        m1 = UnsizedMarker(ship)
        R = transforms.Affine2D().rotate_deg(-COG[i])
        ship2 = ship.transformed(R)
        m2 = UnsizedMarker(ship2)
        icon.append(m2)
    '''zijian---------------------虚拟船改进------------------------------'''
    if h < m - 1 and vision_value[h] == 1:  # FIXME Zhang Zijian
        '''zijian---------------------end------------------------------'''
        Path = mpath.Path
        ship_vision = Path([[CPA[h][0] - 1, CPA[h][1]], [CPA[h][0] - 1, CPA[h][1] + 3], [CPA[h][0], CPA[h][1] + 6],
                            [CPA[h][0] + 1, CPA[h][1] + 3], [CPA[h][0] + 1, CPA[h][1]], [CPA[h][0], CPA[h][1]]],
                           [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY])
        m1 = UnsizedMarker(ship_vision)
        R = transforms.Affine2D().rotate_deg(-COG_area[h])
        ship2_vision = ship_vision.transformed(R)
        m2 = UnsizedMarker(ship2_vision)
        icon.append(m2)
    return icon

if __name__ == "__main__":
    Lat, Lon, COG, SOG, DCPA, TCPA, DCPA_lim, TCPA_lim, lat_wp, lon_wp = AISDataGet.RedisDataGet()
    position_all_x, position_all_y, wpx, wpy = AISDataGet.LatLon2Cartesian(Lat, Lon, lat_wp, lon_wp)
    waypoint, x_tra, y_tra, n, m, h, label_ship, label_waypoint, warn, WpCog, index_value, CPA, pos_vision, COG_area, vision_value = WaypointGet.WaypointGet(Lat, Lon, COG, SOG, DCPA, TCPA, DCPA_lim, TCPA_lim, position_all_x, position_all_y, wpx, wpy)
    icon = shipmodle(COG, x_tra, y_tra, n, CPA, h, m, COG_area, vision_value)
    # 显示
    plt.figure(dpi=128)
    plt.grid(True)
    plt.grid(color='black', linestyle='--', linewidth=0.3, alpha=0.3)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim(-30, 30)
    plt.ylim(-30, 30)
    for i in range(0, n):
        plt.scatter(x_tra[i], y_tra[i], marker=icon[i], s=10, facecolor="none", edgecolors="black")
    plt.show()
