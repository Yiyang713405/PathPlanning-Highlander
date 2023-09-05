import redis
import json
import WGS84toCartesian


def RedisDataWrite():
    # 写数据
    pool = redis.ConnectionPool(host='127.0.0.1', port=6379, db=0)
    # pool = redis.ConnectionPool(host='169.254.197.145', port=6379, db=0)
    # 从连接池中获取一个 Redis 连接
    r = redis.StrictRedis(connection_pool=pool)
    # 创建字典，并将其转换为 JSON 格式
    CpaTcpaLim = {'ActivateTargetOnCollisionAlert': True, 'Active': True, 'CPA_lim_NM': 0.5, 'TCPA_lim_sec': 600}
    # TractionPoint = {'Lat': 31.092, 'Lon': 125.307, 'wpActiveIndex': 1}
    # TractionPoint = {'Lat': 32.4, 'Lon': 125.4, 'wpActiveIndex': 1}

    json_data1 = json.dumps(CpaTcpaLim)
    # json_data2 = json.dumps(TractionPoint)
    # 将 JSON 数据写入 Redis 的一个字符串类型的键中
    r.set('Common.Command.CpaTcpa', json_data1)
    # r.set('ECDIS.Status.WptPoint.Active', json_data2)


def RedisDataGet():
    data = redis.ConnectionPool(host='127.0.0.1', port=6379, db=0)
    # data = redis.ConnectionPool(host='169.254.197.145', port=6379, db=0)
    r = redis.StrictRedis(connection_pool=data)
    lat_all = []
    lon_all = []
    COG_all = []
    SOG_all = []
    DCPA_all = []
    TCPA_all = []
    lat_wp = []
    lon_wp = []

    data_ship = []
    for i in r.scan_iter("AIS.Target*"):
        data_ship.append(i)
        res = sorted(data_ship)  # 按照字母排列

    for i in range(len(data_ship)):
        res_obs1 = r.get(res[i])
        res_obs1 = res_obs1.decode()  # 二进制转字符串
        res_obs1 = json.loads(res_obs1)  # 字符串转字典
        state1 = res_obs1['Position']
        Lat = (state1['Lat'])
        Lon = (state1['Lon'])
        COG1 = res_obs1['COG']
        SOG1 = res_obs1['SOG']
        lat_all.append(Lat)
        lon_all.append(Lon)
        COG_all.append(COG1)
        SOG_all.append(SOG1)
    for i in range(len(data_ship)-1):
        res_obs1 = r.get(res[i])
        res_obs1 = res_obs1.decode()  # 二进制转字符串
        res_obs1 = json.loads(res_obs1)  # 字符串转字典
        DCPA1 = res_obs1['CPA']
        TCPA1 = res_obs1['TCPA']
        DCPA_all.append(DCPA1)
        TCPA_all.append(TCPA1)
    res_obs1 = r.get('Common.Command.CpaTcpa')
    res_obs1 = res_obs1.decode()  # 二进制转字符串
    res_obs1 = json.loads(res_obs1)  # 字符串转字典
    DCPA_lim = res_obs1['CPA_lim_NM']
    TCPA_lim = res_obs1['TCPA_lim_sec']
    wppoint = []    # 牵引点数据
    for i in r.scan_iter("ECDIS.Status.WptPoint.Active*"):
        wppoint.append(i)
        res = sorted(wppoint)  # 按照字母排列

    for i in range(len(wppoint)):
        res_obs1 = r.get(res[i])
        res_obs1 = res_obs1.decode()  # 二进制转字符串
        res_obs1 = json.loads(res_obs1)  # 字符串转字典
        lat = res_obs1['Lat']
        lon = res_obs1['Lon']
        lat_wp.append(lat)
        lon_wp.append(lon)

    return lat_all, lon_all, COG_all, SOG_all, DCPA_all, TCPA_all, DCPA_lim, TCPA_lim, lat_wp, lon_wp


def LatLon2Cartesian(Lat, Lon, lat_wp, lon_wp):
    TargetNum = len(Lat)
    WppointNum = len(lat_wp)
    position_all_x = []
    position_all_y = []
    wpx = []
    wpy = []
    ref_lat = Lat[TargetNum - 1]
    ref_lon = Lon[TargetNum - 1]
    for i in range(TargetNum):
        PC = WGS84toCartesian.PositionConvert()
        lat = Lat[i]
        lon = Lon[i]
        x, y = PC.GPStoXY(lat, lon, ref_lat, ref_lon)
        position_all_x.append(x)
        position_all_y.append(y)
    for j in range(WppointNum):
        PC = WGS84toCartesian.PositionConvert()
        wplat = lat_wp[j]
        wplon = lon_wp[j]
        x, y = PC.GPStoXY(wplat, wplon, ref_lat, ref_lon)
        wpx.append(x)
        wpy.append(y)
    return position_all_x, position_all_y, wpx, wpy


if __name__ == "__main__":
    RedisDataWrite()
    Lat, Lon, COG, SOG, DCPA, TCPA, DCPA_lim, TCPA_lim, lat_wp, lon_wp = RedisDataGet()  # CPA, TCPA, CPA_lim, TCPA_lim
    print(Lat, Lon, COG, SOG, DCPA, TCPA, DCPA_lim, TCPA_lim, lat_wp, lon_wp)
    position_all_x, position_all_y, wpx, wpy = LatLon2Cartesian(Lat, Lon, lat_wp, lon_wp)
    print(Lat, Lon, COG, SOG, position_all_x, position_all_y, wpx, wpy)
