"""
经纬度计算
"""
import math


# 测试通过
def DDD2DMS(number):
    D = number // 1
    temp = number % 1
    M = (temp * 60) // 1
    temp = (temp * 60) % 1
    S = (temp * 60)
    return D + (M / 100) + (S / 10000)


# 求连点的经纬度 返回0-360 顺时针为正
def angleFromCoordinate(long1, lat1, long2, lat2):
    lat1 = math.radians(DDD2DMS(lat1))
    lat2 = math.radians(DDD2DMS(lat2))
    long1 = math.radians(DDD2DMS(long1))
    long2 = math.radians(DDD2DMS(long2))
    y = math.sin(long2 - long1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * \
        math.cos(lat2) * math.cos(long2 - long1)
    deltaLon = long2 - long1
    theta = math.atan2(y, x)
    theta = math.degrees(theta)
    theta = (theta + 360) % 360
    return theta


# 一直两点经纬度求两点的距离单位，返回单位厘米
def distanceFromCoordinate(lon1, lat1, lon2, lat2):  # 经度1，纬度1，经度2，纬度2 （十进制度数）
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # 将十进制度数转化为弧度
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine公式
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * \
        math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # 地球平均半径，单位为公里
    return c * r * 1000

# 已知一点的经纬度和移动方向与距离，求终点的经纬度


def one_point_diatance_to_end(lng, lat, brng, d):
    R = 6378.1  # Radius of the Earth
    brng = math.radians(brng)  # Bearing is 90 degrees converted to radians.
    d = d / 1000 # Distance in km

    # lat2  52.20444 - the lat result I'm hoping for
    # lon2  0.36056 - the long result I'm hoping for.

    lat1 = math.radians(lat)  # Current lat point converted to radians
    lon1 = math.radians(lng)  # Current long point converted to radians

    lat2 = math.asin(math.sin(lat1) * math.cos(d / R) +
                     math.cos(lat1) * math.sin(d / R) * math.cos(brng))

    lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / R) * math.cos(lat1),
                             math.cos(d / R) - math.sin(lat1) * math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    return [lon2, lat2]


if __name__ == '__main__':
    temp = angleFromCoordinate(114.316966, 30.576768, 114.397346, 30.58709)
    print(temp)
    temp = distanceFromCoordinate(114.316966, 30.576768, 114.397346, 30.58709)
    print(temp)
    temp = one_point_diatance_to_end(114.316966, 30.576768, 78.7, 777974)
    print(temp)
