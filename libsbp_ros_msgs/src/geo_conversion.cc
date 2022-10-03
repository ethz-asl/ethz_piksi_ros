#include "libsbp_ros_msgs/geo_conversion.h"

#include <ros/assert.h>
#include <ros/node_handle.h>

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
  ROS_INFO("Updated ENU frame with WGS84 (lat, lon, alt): (%.6f, %.6f, %.6f)",
           lat, lon, alt);
}

void GeographicConversion::initFromRosParam(const std::string& prefix) {
  ros::NodeHandle nh;
  if (!nh.ok() || !nh.hasParam(prefix)) {
    ROS_WARN("[GeoConversion] No Geodetic Transformations found.");
    return;
  }
  XmlRpc::XmlRpcValue yaml_raw_data;
  nh.getParam("/geotf", yaml_raw_data);

  auto& frame_definitions = yaml_raw_data["Frames"];
  for (auto it = frame_definitions.begin(); it != frame_definitions.end();
       ++it) {
    const std::string frame_name = it->first;
    auto& xmlnode = it->second;

    std::string frame_type = xmlnode["Type"];
    if (frame_type == "ENUOrigin") {
      if (!xmlnode.hasMember("LatOrigin") || !xmlnode.hasMember("LonOrigin") ||
          !xmlnode.hasMember("AltOrigin")) {
        ROS_WARN_STREAM("[GeoConversion] Ignoring frame "
                        << frame_type
                        << ": ENU origin needs LatOrigin, LonOrigin and "
                           "AltOrigin setting.");
        continue;
      }
      double latOrigin = xmlnode["LatOrigin"];
      double lonOrigin = xmlnode["LonOrigin"];
      double altOrigin = xmlnode["AltOrigin"];

      setEnuFrame(latOrigin, lonOrigin, altOrigin);
    }
  }
}

}  // namespace libsbp_ros_msgs