# 可视化规则：选择坐标原点并整理模型、轨迹和状态的显示参数。

from dataclasses import dataclass
import math

from .geo import lat_lon_to_local


@dataclass(frozen=True)
class Origin:
    lat: float
    lon: float


def select_origin(
    model_origin_lat,
    model_origin_lon,
    model_origin_valid,
    configured_origin_lat,
    configured_origin_lon,
    configured_origin_valid,
):
    # 按模型原点、配置原点和有效标志选择可用的可视化原点。
    if _origin_is_valid(model_origin_lat, model_origin_lon, model_origin_valid):
        return Origin(float(model_origin_lat), float(model_origin_lon))
    if _origin_is_valid(
        configured_origin_lat,
        configured_origin_lon,
        configured_origin_valid,
    ):
        return Origin(float(configured_origin_lat), float(configured_origin_lon))
    return None


def _origin_is_valid(lat, lon, enabled):
    # 判断候选原点是否启用且处于合法经纬度范围。
    if not enabled:
        return False
    try:
        lat_lon_to_local(float(lat), float(lon), float(lat), float(lon))
    except (TypeError, ValueError):
        return False
    return True


def segment_local_points(segment, origin):
    # 把任务段的起终点转换为相对原点的局部坐标。
    if origin is None:
        return None
    try:
        start = lat_lon_to_local(
            origin.lat,
            origin.lon,
            float(segment.start_lat),
            float(segment.start_lon),
        )
        end = lat_lon_to_local(
            origin.lat,
            origin.lon,
            float(segment.end_lat),
            float(segment.end_lon),
        )
    except (AttributeError, TypeError, ValueError):
        return None
    return start, end


def vehicle_color(fixed_valid, coordinate_valid, fresh):
    # 根据RTK固定解、坐标有效性和新鲜度选择车辆显示颜色。
    if not coordinate_valid or not fresh:
        return 1.0, 0.15, 0.1, 1.0
    if not fixed_valid:
        return 1.0, 0.75, 0.1, 1.0
    return 0.1, 0.9, 0.2, 1.0


def is_rtk_fresh(gga_age_sec, receive_age_sec, timeout_sec):
    # 判断GGA年龄和接收年龄是否都在允许超时范围内。
    # 同时检查RTK数据年龄和本节点最后接收消息的时间。
    if timeout_sec <= 0.0 or not math.isfinite(timeout_sec):
        raise ValueError("timeout must be finite and positive")
    ages = (gga_age_sec, receive_age_sec)
    if any(not math.isfinite(age) or age < 0.0 for age in ages):
        return False
    return max(ages) <= timeout_sec


def model_group_geometry(group):
    # 提取清扫模型分组的多边形几何点，供Marker生成使用。
    # 返回模型点、闭合子区域线段和以米为单位的连接线对。
    points_by_id = {}
    ordered_points = []
    for point in getattr(group, "points", []):
        try:
            x = float(point.x_cm) / 100.0
            y = float(point.y_cm) / 100.0
        except (AttributeError, TypeError, ValueError):
            continue
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        coordinate = (x, y)
        points_by_id[str(point.id)] = coordinate
        ordered_points.append(coordinate)

    areas = []
    for area in getattr(group, "sub_areas", []):
        line = [
            points_by_id[point_id]
            for point_id in getattr(area, "point_ids", [])
            if point_id in points_by_id
        ]
        if len(line) >= 2:
            if line[0] != line[-1]:
                line.append(line[0])
            areas.append(line)

    connectors = []
    for connector in getattr(group, "connectors", []):
        start = points_by_id.get(str(connector.start_point_id))
        end = points_by_id.get(str(connector.end_point_id))
        if start is not None and end is not None:
            connectors.append((start, end))

    return {
        "points": ordered_points,
        "areas": areas,
        "connectors": connectors,
    }
