#ifndef CLEANBOT_RTK__VEHICLE_CENTER_TRANSFORM_HPP_
#define CLEANBOT_RTK__VEHICLE_CENTER_TRANSFORM_HPP_

// 文件作用：声明 GNSS 天线坐标到车辆几何中心坐标的航向相关偏移变换。
namespace cleanbot {
namespace rtk {

// 经纬度坐标及其是否计算有效。
struct CenterPoint {
  double lat{0.0};
  double lon{0.0};
  bool valid{false};
};

class VehicleCenterTransform {
 public:
  // 按航向、前向和右侧天线偏移计算车辆中心坐标。
  CenterPoint compute(
      double raw_lat,
      double raw_lon,
      double heading_deg,
      double along_heading_m = 0.10,
      double right_m = 0.18) const;

 private:
  // 从起点沿指定方位和距离计算球面目标坐标。
  static CenterPoint destination(
      double lat, double lon, double bearing_deg, double distance_m);
  // 将经纬度按接口精度保留到小数点后八位。
  static double round8(double value);
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__VEHICLE_CENTER_TRANSFORM_HPP_
