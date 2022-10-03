#include "libsbp_ros_msgs/geo_conversion.h"

#include <ros/assert.h>

namespace libsbp_ros_msgs {

GeographicConversion::GeographicConversion() {}

bool GeographicConversion::convertEnuToEcef(const Eigen::Vector3d& enu,
                                            Eigen::Vector3d* ecef) {
  ROS_ASSERT(ecef);
  Eigen::Vector3d wgs84;
  if (convertEnuToWgs84(enu, &wgs84)) {
    convertWgs84ToEcef(wgs84, ecef);
    return true;
  } else {
    return false;
  }
}

bool GeographicConversion::convertEcefToEnu(const Eigen::Vector3d& ecef,
                                            Eigen::Vector3d* enu) {
  ROS_ASSERT(enu);
  Eigen::Vector3d wgs84;
  convertEcefToWgs84(ecef, &wgs84);
  return convertWgs84ToEnu(wgs84, enu);
}

bool GeographicConversion::convertEnuToWgs84(const Eigen::Vector3d& enu,
                                             Eigen::Vector3d* wgs84) {
  ROS_ASSERT(wgs84);
  if (enu_.has_value()) {
    enu_.value().Reverse(enu.x(), enu.y(), enu.z(), wgs84->x(), wgs84->y(),
                         wgs84->z());
    return true;
  } else {
    return false;
  }
}

bool GeographicConversion::convertWgs84ToEnu(const Eigen::Vector3d& wgs84,
                                             Eigen::Vector3d* enu) {
  ROS_ASSERT(enu);
  if (enu_.has_value()) {
    enu_.value().Forward(wgs84.x(), wgs84.y(), wgs84.z(), enu->x(), enu->y(),
                         enu->z());
    return true;
  } else {
    return false;
  }
}

void GeographicConversion::convertEcefToWgs84(const Eigen::Vector3d& ecef,
                                              Eigen::Vector3d* wgs84) {
  ROS_ASSERT(wgs84);
  ecef_.Reverse(ecef.x(), ecef.y(), ecef.z(), wgs84->x(), wgs84->y(),
                wgs84->z());
}

void GeographicConversion::convertWgs84ToEcef(const Eigen::Vector3d& wgs84,
                                              Eigen::Vector3d* ecef) {
  ROS_ASSERT(ecef);
  ecef_.Forward(wgs84.x(), wgs84.y(), wgs84.z(), ecef->x(), ecef->y(),
                ecef->z());
}

bool GeographicConversion::hasEnuFrame() { return enu_.has_value(); }

void GeographicConversion::setEnuFrame(double lat, double lon, double alt) {
  enu_.emplace(lat, lon, alt, GeographicLib::Geocentric::WGS84());
}

}  // namespace libsbp_ros_msgs