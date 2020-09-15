#ifndef LIBSBP_ROS_MSGS_ROS_CONVERSION_H_
#define LIBSBP_ROS_MSGS_ROS_CONVERSION_H_

#include <libsbp/navigation.h>
#include <ros/assert.h>
#include <ros/time.h>
#include <Eigen/Dense>

// Manually implemented ROS conversions.
namespace libsbp_ros_msgs {

const double kFromMilli = 1.0e-3;
const double kFromMilliSq = 1.0e-6;

ros::Time convertUtcTimeToRosTime(const msg_utc_time_t& utc_msg);
ros::Time convertGpsTimeToRosTimeWithLeapSecondOffset(
    const msg_gps_time_t& gps_msg);
ros::Time convertGpsTimeToRosTimeWithLeapSecondOffset(
    const uint16_t wn, const uint32_t tow, const int32_t ns_residual);
ros::Time convertGpsTimeToUtcRosTime(const msg_gps_time_t& gps_msg,
                                     const uint32_t leap_seconds);
ros::Time convertGpsTimeToUtcRosTime(const uint16_t wn, const uint32_t tow,
                                     const int32_t ns_residual,
                                     const uint32_t leap_seconds);
ros::Time convertTowToRosTime(const uint32_t tow, const uint32_t leap_seconds);
ros::Time convertTowTowfToRosTime(const uint32_t tow, const uint8_t tow_f,
                                  const uint32_t leap_seconds);

Eigen::Matrix3d getRotationEcefToEnu(const double lat_deg,
                                     const double lon_deg);
Eigen::Matrix3d getRotationEcefToNed(const double lat_deg,
                                     const double lon_deg);

template <class CartesianPointIn, class CartesianPointOut>
inline void convertCartesianPoint(const CartesianPointIn& in,
                                  CartesianPointOut* out) {
  ROS_ASSERT(out);
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

template <class CartesianPointIn>
inline void convertCartesianPoint(const CartesianPointIn& in,
                                  Eigen::Vector3d* out) {
  ROS_ASSERT(out);
  out->x() = in.x;
  out->y() = in.y;
  out->z() = in.z;
}

template <class CartesianVectorIn, class CartesianVectorOut>
inline void convertCartesianVector(const CartesianVectorIn& in,
                                   CartesianVectorOut* out) {
  ROS_ASSERT(out);
  out->x = in.x * kFromMilli;
  out->y = in.y * kFromMilli;
  out->z = in.z * kFromMilli;
}

template <class NedVectorIn, class NedPositionOut>
inline void convertNedVector(const NedVectorIn& in, NedPositionOut* out) {
  ROS_ASSERT(out);
  out->x = in.n * kFromMilli;
  out->y = in.e * kFromMilli;
  out->z = in.d * kFromMilli;
}

template <class NedAccuracyIn, class NedCovOut>
inline void convertNedAccuracyToNedCov(const NedAccuracyIn& in,
                                       NedCovOut* out) {
  ROS_ASSERT(out);
  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dRow;
  Matrix3dRow cov = Matrix3dRow::Zero();
  cov(0, 0) = in.h_accuracy * in.h_accuracy * kFromMilliSq;
  cov(1, 1) = cov(0, 0);
  cov(2, 2) = in.v_accuracy * in.v_accuracy * kFromMilliSq;
  Matrix3dRow::Map(out->data()) = cov;
}

template <class Wgs84PointIn, class Wgs84PointOut>
inline void convertWgs84Point(const Wgs84PointIn& in, Wgs84PointOut* out) {
  ROS_ASSERT(out);
  out->latitude = in.lat;
  out->longitude = in.lon;
  out->altitude = in.height;
}

template <class Wgs84PointIn>
inline void convertWgs84Point(const Wgs84PointIn& in, Eigen::Vector3d* out) {
  ROS_ASSERT(out);
  out->x() = in.lat;
  out->y() = in.lon;
  out->z() = in.height;
}

template <class CartesianCovIn, class CartesianCovOut>
inline void convertCartesianCov(const CartesianCovIn& in,
                                CartesianCovOut* out) {
  ROS_ASSERT(out);
  // First row.
  (*out)[0] = in.cov_x_x;
  (*out)[1] = in.cov_x_y;
  (*out)[2] = in.cov_x_z;
  // Second row.
  (*out)[3] = in.cov_x_y;
  (*out)[4] = in.cov_y_y;
  (*out)[5] = in.cov_y_z;
  // Third row.
  (*out)[6] = in.cov_x_z;
  (*out)[7] = in.cov_y_z;
  (*out)[8] = in.cov_z_z;
}

template <class CartesianCovIn>
inline void convertCartesianCov(const CartesianCovIn& in,
                                Eigen::Matrix3d* out) {
  ROS_ASSERT(out);
  std::vector<double> cov(9);
  convertCartesianCov<CartesianCovIn, std::vector<double>>(in, &cov);
  *out = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(cov.data());
}

template <class CartesianCovIn, class CartesianCovOut>
inline void convertCartesianCovMm(const CartesianCovIn& in,
                                  CartesianCovOut* out) {
  convertCartesianCov<CartesianCovIn, CartesianCovOut>(in, out);
  for (auto& cov : *out) {
    cov *= kFromMilliSq;
  }
}

template <class CovarianceNedIn, class CovarianceNedOut>
inline void convertNedCov(const CovarianceNedIn& in, CovarianceNedOut* out) {
  ROS_ASSERT(out);
  // First row.
  (*out)[0] = in.cov_n_n;
  (*out)[1] = in.cov_n_e;
  (*out)[2] = in.cov_n_d;
  // Second row.
  (*out)[3] = in.cov_n_e;
  (*out)[4] = in.cov_e_e;
  (*out)[5] = in.cov_e_d;
  // Third row.
  (*out)[6] = in.cov_n_d;
  (*out)[7] = in.cov_e_d;
  (*out)[8] = in.cov_d_d;
}

template <class CovarianceNedIn, class CovarianceNedOut>
inline void convertNedCovMm(const CovarianceNedIn& in, CovarianceNedOut* out) {
  convertNedCov<CovarianceNedIn, CovarianceNedOut>(in, out);
  for (auto& cov : *out) {
    cov *= kFromMilliSq;
  }
}

}  // namespace libsbp_ros_msgs

#endif  // LIBSBP_ROS_MSGS_ROS_CONVERSION_H_
