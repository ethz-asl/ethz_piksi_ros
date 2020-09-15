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

  const double phi = lat_deg * kDegToRad;
  const double s_phi = std::sin(phi);
  const double c_phi = std::cos(phi);

  const double lambda = lon_deg * kDegToRad;
  const double s_lambda = std::sin(lambda);
  const double c_lambda = std::cos(lambda);

  Eigen::Matrix3d R_ENU_ECEF;
  R_ENU_ECEF(0, 0) = -s_lambda;
  R_ENU_ECEF(0, 1) = c_lambda;
  R_ENU_ECEF(0, 2) = 0.0;

  R_ENU_ECEF(1, 0) = -s_phi * c_lambda;
  R_ENU_ECEF(1, 1) = -s_phi * s_lambda;
  R_ENU_ECEF(1, 2) = c_phi;

  R_ENU_ECEF(2, 0) = c_phi * c_lambda;
  R_ENU_ECEF(2, 1) = c_phi * s_lambda;
  R_ENU_ECEF(2, 2) = s_phi;

  return R_ENU_ECEF;
}

Eigen::Matrix3d getRotationEcefToNed(const double lat_deg,
                                     const double lon_deg) {
  auto R_ENU_ECEF = getRotationEcefToEnu(lat_deg, lon_deg);
  return (Eigen::Matrix3d() << R_ENU_ECEF.row(1), R_ENU_ECEF.row(0),
          -R_ENU_ECEF.row(2))
      .finished();
}

}  // namespace libsbp_ros_msgs
