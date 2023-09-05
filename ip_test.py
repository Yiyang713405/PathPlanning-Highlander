import math
import numpy as np

import redis
import json

# 准备数据
rx_for_redis = [0.23987886282656928, 2.1296364126873977, 5.8343418331546655, 11.275932850223796, 22.52421864507096, 788.9973583788336]
ry_for_redis = [24.459345574135956, 73.1422892501623, 121.52177943250231, 169.6108678843253, 241.22854804927317, 2293.5279617555675]

data = {
    "OSExtraCollisionAvoidancePara": {
        "AimCourse": 10,
        "CrsChange": 10,
        "RecoverCrs": 10
    },
    "ZAvoidingRoute": []
}

for i in range(len(rx_for_redis)):
    item = {
        "posX": rx_for_redis[i],
        "posY": ry_for_redis[i]
    }
    data["ZAvoidingRoute"].append(item)

# 转换为 JSON 字符串
json_str = json.dumps(data)

# 连接 Redis
r = redis.Redis(host='localhost', port=6379, db=0)

# 写入 Redis
r.set('data', json_str)
