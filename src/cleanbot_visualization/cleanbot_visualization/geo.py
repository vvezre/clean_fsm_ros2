# 地理坐标工具：校验WGS84坐标并转换为局部东-北平面坐标。

import math


EARTH_RADIUS_M = 6378137.0


def _validate_coordinate(lat, lon):
    # 校验纬度、经度均为有限值且处于WGS84合法范围。
    if not math.isfinite(lat) or not math.isfinite(lon):
        raise ValueError("coordinate must be finite")
    if not -90.0 <= lat <= 90.0:
        raise ValueError("latitude is out of range")
    if not -180.0 <= lon <= 180.0:
        raise ValueError("longitude is out of range")


def lat_lon_to_local(origin_lat, origin_lon, lat, lon):
    # 把WGS84经纬度转换为相对原点的东向、北向米制坐标。
    _validate_coordinate(origin_lat, origin_lon)
    _validate_coordinate(lat, lon)
    mean_lat = math.radians((origin_lat + lat) * 0.5)
    east = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(mean_lat)
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return float(east), float(north)


def heading_to_quaternion(heading_deg):
    # 把以度为单位的航向角转换为平面旋转四元数。
    # 将罗盘航向角（北为0、顺时针增加）转换为ROS ENU偏航角。
    if not math.isfinite(heading_deg):
        raise ValueError("heading must be finite")
    normalized_heading = heading_deg % 360.0
    yaw = math.radians(90.0 - normalized_heading)
    return (
        0.0,
        0.0,
        math.sin(yaw * 0.5),
        math.cos(yaw * 0.5),
    )
