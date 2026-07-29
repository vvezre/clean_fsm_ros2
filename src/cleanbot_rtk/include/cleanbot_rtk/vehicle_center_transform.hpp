#ifndef CLEANBOT_RTK__VEHICLE_CENTER_TRANSFORM_HPP_
#define CLEANBOT_RTK__VEHICLE_CENTER_TRANSFORM_HPP_

namespace cleanbot {
namespace rtk {

struct CenterPoint {
  double lat{0.0};
  double lon{0.0};
  bool valid{false};
};

class VehicleCenterTransform {
 public:
  CenterPoint compute(
      double raw_lat,
      double raw_lon,
      double heading_deg,
      double along_heading_m = 0.10,
      double right_m = 0.18) const;

 private:
  static CenterPoint destination(
      double lat, double lon, double bearing_deg, double distance_m);
  static double round8(double value);
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__VEHICLE_CENTER_TRANSFORM_HPP_
