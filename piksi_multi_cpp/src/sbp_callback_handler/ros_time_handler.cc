#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"

#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/node_handle.h>
#include <cmath>
#include <functional>

namespace piksi_multi_cpp {
namespace s = std::placeholders;
namespace lrm = libsbp_ros_msgs;

RosTimeHandler::RosTimeHandler(const std::shared_ptr<sbp_state_t>& state)
    : gps_time_handler_{std::bind(&RosTimeHandler::callbackToGpsTime, this,
                                  s::_1, s::_2),
                        SBP_MSG_GPS_TIME, state},
      utc_time_handler_{
          std::bind(&RosTimeHandler::callbackToUtcTime, this, s::_1, s::_2),
          SBP_MSG_UTC_TIME, state} {
  // Check if user wants to stamp data with GPS time.
  ros::NodeHandle nh_node("~");
  // TODO(rikba): Instead of setting a member variable it would be nicer to have
  // the lambda callbacks optional.
  nh_node.getParam("use_gps_time", use_gps_time_);
  ROS_INFO_COND(use_gps_time_, "Using GPS time stamping.");
  ROS_WARN_COND(!use_gps_time_, "Using ros::Time::now() to stamp data.");
}

bool RosTimeHandler::utcTimeReady() const { return leap_seconds_.has_value(); }

// This callback should always arrive before UTC time.
void RosTimeHandler::callbackToGpsTime(const msg_gps_time_t& msg,
                                       const uint8_t len) {
  if (((msg.flags >> 0) & 0x7) == 0) {  // No GNSS time.
    last_gps_time_.reset();
    tow_to_utc_.reset();
    return;
  }
  // Cache GPS time without leap seconds to calculate leap seconds.
  last_gps_time_ =
      std::make_optional(lrm::convertGpsTimeToRosTimeWithLeapSecondOffset(msg));
  // Calculate UTC time and cache tow, GPS time pair to lookup time stamp.
  if (leap_seconds_.has_value())
    tow_to_utc_ = std::make_optional(std::make_pair(
        msg.tow, lrm::convertGpsTimeToUtcRosTime(msg, leap_seconds_.value())));
}

void RosTimeHandler::callbackToUtcTime(const msg_utc_time_t& msg,
                                       const uint8_t len) {
  if (((msg.flags >> 0) & 0x7) == 0) {  // No GNSS time.
    return;
  }
  // Calculate new GNSS leap seconds.
  if (!last_gps_time_.has_value()) {
    ROS_WARN("No GPS time received.");
    return;
  }
  uint32_t new_leap_s = std::round(
      (last_gps_time_.value() - lrm::convertUtcTimeToRosTime(msg)).toSec());
  last_gps_time_.reset();  // Invalidate GPS time.
  if (leap_seconds_.has_value() && leap_seconds_.value() != new_leap_s) {
    ROS_WARN(
        "GPS leap seconds have changed from: %d to %d. Either you are working "
        "during new years evening or a bug occured.",
        leap_seconds_.value(), new_leap_s);
  }
  ROS_INFO_COND(!leap_seconds_.has_value(), "GPS leap seconds: %d", new_leap_s);
  leap_seconds_ = std::make_optional(new_leap_s);
}

ros::Time RosTimeHandler::lookupTime(const uint32_t tow) const {
  if (!use_gps_time_) {
  } else if (tow_to_utc_.has_value() && tow_to_utc_.value().first == tow) {
    return tow_to_utc_.value().second;  // Can lookup ns timestamp.
  } else if (leap_seconds_.has_value()) {
    ROS_WARN("Cannot lookup GPS time. Converting ms precision tow to UTC.");
    return lrm::convertTowToRosTime(tow, leap_seconds_.value());
  } else {
    ROS_WARN_THROTTLE(
        10.0,
        "Failed to convert tow to GPS time stamp. Stamping data with "
        "ros::Time::now()");
  }
  return ros::Time::now();
}

ros::Time RosTimeHandler::lookupTime(const uint32_t tow,
                                     const uint8_t tow_f) const {
  if (!use_gps_time_) {
  } else if (tow >> 31) {
    ROS_WARN_THROTTLE(
        10.0,
        "High bit of tow set. Time invalid or unknown. Setting stamp to "
        "ros::Time::now().");
  } else if (leap_seconds_.has_value()) {
    return lrm::convertTowTowfToRosTime(tow, tow_f, leap_seconds_.value());
  } else {
    ROS_WARN_THROTTLE(
        10.0,
        "Failed to convert tow, tow_f to GPS time stamp. Stamping data with "
        "ros::Time::now()");
  }
  return ros::Time::now();
}

ros::Time RosTimeHandler::convertGpsTime(const uint16_t wn, const uint32_t tow,
                                         const int32_t ns_residual) const {
  if (!use_gps_time_) {
  } else if (leap_seconds_.has_value()) {
    return lrm::convertGpsTimeToUtcRosTime(wn, tow, ns_residual,
                                           leap_seconds_.value());
  } else {
    ROS_WARN_THROTTLE(
        10.0,
        "Unkown leap seconds. Failed to convert wn, tow, ns_residual to UTC "
        "time stamp. Stamping data with ros::Time::now()");
  }
  return ros::Time::now();
}

}  // namespace piksi_multi_cpp
