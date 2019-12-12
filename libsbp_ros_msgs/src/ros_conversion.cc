#include "libsbp_ros_msgs/ros_conversion.h"

#include <cmath>

#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

namespace libsbp_ros_msgs {

namespace bpt = boost::posix_time;
namespace bg = boost::gregorian;

const bpt::ptime kEpochStart(bg::date(1970, 1, 1));
const bpt::ptime kGpsStart(bg::date(1980, 1, 6));
const double kMilliToNano = 10.0e6;

ros::Time convertUtcTimeToRosTime(const msg_utc_time_t& utc_msg) {
  bpt::ptime t_utc(bg::date(utc_msg.year, utc_msg.month, utc_msg.day),
                   bpt::hours(utc_msg.hours) + bpt::minutes(utc_msg.minutes) +
                       bpt::seconds(utc_msg.seconds) +
                       bpt::nanosec(utc_msg.ns));
  bpt::time_duration d_utc = t_utc - kEpochStart;

  return ros::Time(d_utc.total_seconds(), d_utc.fractional_seconds());
}

ros::Time convertGpsTimeToRosTimeWithLeapSecondOffset(
    const msg_gps_time_t& gps_msg) {
  return convertGpsTimeToRosTimeWithLeapSecondOffset(gps_msg.wn, gps_msg.tow,
                                                     gps_msg.ns_residual);
}

ros::Time convertGpsTimeToRosTimeWithLeapSecondOffset(
    const uint16_t wn, const uint32_t tow, const int32_t ns_residual) {
  bpt::ptime last_sunday(kGpsStart + bg::weeks(wn));
  bpt::milliseconds ms(tow);
  bpt::nanosec ns(ns_residual);
  // We do not account for leap seconds here!
  bpt::ptime t_utc = last_sunday + (ms + ns);
  bpt::time_duration d_utc = t_utc - kEpochStart;
  return ros::Time(d_utc.total_seconds(), d_utc.fractional_seconds());
}

ros::Time convertGpsTimeToUtcRosTime(const msg_gps_time_t& gps_msg,
                                     const uint32_t leap_seconds) {
  return convertGpsTimeToUtcRosTime(gps_msg.wn, gps_msg.tow,
                                    gps_msg.ns_residual, leap_seconds);
}

ros::Time convertGpsTimeToUtcRosTime(const uint16_t wn, const uint32_t tow,
                                     const int32_t ns_residual,
                                     const uint32_t leap_seconds) {
  return convertGpsTimeToRosTimeWithLeapSecondOffset(wn, tow, ns_residual) -
         ros::Duration(leap_seconds, 0);
}

ros::Time convertTowToRosTime(const uint32_t tow, const uint32_t leap_seconds) {
  return convertTowTowfToRosTime(tow, 0, leap_seconds);
}

ros::Time convertTowTowfToRosTime(const uint32_t tow, const uint8_t tow_f,
                                  const uint32_t leap_seconds) {
  bg::date today(bg::day_clock::universal_day());
  bpt::ptime last_sunday(today, bpt::hours(-24 * today.day_of_week()));
  bpt::milliseconds ms(tow);
  bpt::nanosec ns(
      static_cast<uint32_t>(std::round(tow_f / 256.0 * kMilliToNano)));
  bpt::ptime t_utc = last_sunday + (ms + ns);
  bpt::time_duration d_utc = t_utc - kEpochStart;

  return ros::Time(d_utc.total_seconds(), d_utc.fractional_seconds()) -
         ros::Duration(leap_seconds, 0);
}

Eigen::Matrix3d getRotationEcefToEnu(const double lat_deg,
                                     const double lon_deg) {
  // https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
  const double kDegToRad = M_PI / 180;
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double s_lat = std::sin(lat);
  const double c_lat = std::cos(lat);
  const double s_lon = std::sin(lon);
  const double c_lon = std::cos(lon);

  Eigen::Matrix3d R_ENU_ECEF;
  R_ENU_ECEF(0,0) = -s_lat;
  R_ENU_ECEF(0,1) =  c_lat;
  R_ENU_ECEF(0,2) = 0.0;

  R_ENU_ECEF(1,0) = -s_lon * c_lat;
  R_ENU_ECEF(1,1) = -s_lon * s_lat;
  R_ENU_ECEF(1,2) = c_lon;

  R_ENU_ECEF(2,0) = c_lon * c_lat;
  R_ENU_ECEF(2,1) = c_lon * s_lat;
  R_ENU_ECEF(2,2) = s_lon;

  return R_ENU_ECEF;
}

}  // namespace libsbp_ros_msgs
