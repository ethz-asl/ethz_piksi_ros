#ifndef LIBSBP_ROS_MSGS_GEO_CONVERSION_H_
#define LIBSBP_ROS_MSGS_GEO_CONVERSION_H_

#include <Eigen/Dense>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <optional>
#include <string>

namespace libsbp_ros_msgs {
class GeographicConversion {
 public:
  GeographicConversion();

  // Converts Position from one input_frame to output_frame.
  // Order x: latitude, east, y: longitude, north, z: altitude, up
  bool convertEnuToEcef(const Eigen::Vector3d& enu, Eigen::Vector3d* ecef);
  bool convertEcefToEnu(const Eigen::Vector3d& ecef, Eigen::Vector3d* enu);

  bool convertEnuToWgs84(const Eigen::Vector3d& enu, Eigen::Vector3d* wgs84);
  bool convertWgs84ToEnu(const Eigen::Vector3d& wgs84, Eigen::Vector3d* enu);

  void convertEcefToWgs84(const Eigen::Vector3d& ecef, Eigen::Vector3d* wgs84);
  void convertWgs84ToEcef(const Eigen::Vector3d& wgs84, Eigen::Vector3d* ecef);

  // Check whether ENU frame is set.
  bool hasEnuFrame();

  // Creates or overwrites the ENU Frame with its origin at the given location
  // (lat, lon, alt), where (lat,lon,alt) are defined w.r.t. WGS84.
  void setEnuFrame(double lat, double lon, double alt);

  void initFromRosParam(const std::string& prefix = "/geotf");

 private:
  // Coordinate frame transformations
  std::optional<GeographicLib::LocalCartesian> enu_;
  const GeographicLib::Geocentric ecef_ = GeographicLib::Geocentric::WGS84();
};

}  // namespace libsbp_ros_msgs

#endif  // LIBSBP_ROS_MSGS_GEO_CONVERSION_H_