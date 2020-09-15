#include <random>

#include <eigen-checks/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

UNITTEST_ENTRYPOINT

#include "libsbp_ros_msgs/ros_conversion.h"

using namespace libsbp_ros_msgs;

const int kSeed = 123456;

std::pair<double, double> createRandomWgs84(std::default_random_engine* re) {
  const double kLatMin = -90.0;
  const double kLatMax = 90.0;
  const double kLonMin = -180.0;
  const double kLonMax = 180.0;

  std::uniform_real_distribution<double> uni_lat(kLatMin, kLatMax);
  std::uniform_real_distribution<double> uni_lon(kLonMin, kLonMax);

  std::pair<double, double> lat_lon;
  lat_lon.first = uni_lat(*re);
  lat_lon.second = uni_lon(*re);

  return lat_lon;
}

TEST(RosConversionTest, RandomEcefEnuConversion) {
  std::default_random_engine re(kSeed);
  for (size_t i = 0; i < 1000; ++i) {
    auto lat_lon = createRandomWgs84(&re);
    auto R_ENU_ECEF = getRotationEcefToEnu(lat_lon.first, lat_lon.second);
    auto R_NED_ECEF = getRotationEcefToNed(lat_lon.first, lat_lon.second);
    // Orthonormal.
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(Eigen::Matrix3d::Identity(),
                                          R_ENU_ECEF * R_ENU_ECEF.transpose()));
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(Eigen::Matrix3d::Identity(),
                                          R_NED_ECEF * R_NED_ECEF.transpose()));

    Eigen::Vector3d ENU_v_E = Eigen::Vector3d::UnitX();
    Eigen::Vector3d ENU_v_N = Eigen::Vector3d::UnitY();
    Eigen::Vector3d ENU_v_U = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d NED_v_E = Eigen::Vector3d::UnitY();
    Eigen::Vector3d NED_v_N = Eigen::Vector3d::UnitX();
    Eigen::Vector3d NED_v_U = -Eigen::Vector3d::UnitZ();

    auto R_ENU_NED = R_ENU_ECEF * R_NED_ECEF.transpose();
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R_ENU_NED * NED_v_E, ENU_v_E));
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R_ENU_NED * NED_v_N, ENU_v_N));
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R_ENU_NED * NED_v_U, ENU_v_U));

    auto R_NED_ENU = R_NED_ECEF * R_ENU_ECEF.transpose();
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R_NED_ENU * ENU_v_E, NED_v_E));
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R_NED_ENU * ENU_v_N, NED_v_N));
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R_NED_ENU * ENU_v_U, NED_v_U));

    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R_ENU_NED, R_NED_ENU.transpose()));
  }
}

TEST(RosConversionTest, GreenichEcefToEnuConversion) {
  auto R_ENU_ECEF = getRotationEcefToEnu(0.0, 0.0);

  // x-axis ECEF points in z direction of ENU at Greenwich.
  // ECEF unit vector in x-direction.
  Eigen::Matrix<double, 3, 1> ECEF_e_ECEF_x = Eigen::Vector3d::UnitX();
  // Converted to ENU frame.
  Eigen::Matrix<double, 3, 1> ENU_e_ECEF_x = R_ENU_ECEF * ECEF_e_ECEF_x;
  // Expected:
  Eigen::Matrix<double, 3, 1> ENU_e_ECEF_x_EXP = Eigen::Vector3d::UnitZ();

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(ENU_e_ECEF_x, ENU_e_ECEF_x_EXP));

  // y-axis ECEF points in east direction of ENU at Greenwich.
  // ECEF unit vector in y-direction.
  Eigen::Matrix<double, 3, 1> ECEF_e_ECEF_y = Eigen::Vector3d::UnitY();
  // Converted to ENU frame.
  Eigen::Matrix<double, 3, 1> ENU_e_ECEF_y = R_ENU_ECEF * ECEF_e_ECEF_y;
  // Expected:
  Eigen::Matrix<double, 3, 1> ENU_e_ECEF_y_EXP = Eigen::Vector3d::UnitX();

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(ENU_e_ECEF_y, ENU_e_ECEF_y_EXP));

  // z-axis ECEF points in north direction of ENU at Greenwich.
  // ECEF unit vector in z-direction.
  Eigen::Matrix<double, 3, 1> ECEF_e_ECEF_z = Eigen::Vector3d::UnitZ();
  // Converted to ENU frame.
  Eigen::Matrix<double, 3, 1> ENU_e_ECEF_z = R_ENU_ECEF * ECEF_e_ECEF_z;
  // Expected:
  Eigen::Matrix<double, 3, 1> ENU_e_ECEF_z_EXP = Eigen::Vector3d::UnitY();

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(ENU_e_ECEF_z, ENU_e_ECEF_z_EXP));
}
