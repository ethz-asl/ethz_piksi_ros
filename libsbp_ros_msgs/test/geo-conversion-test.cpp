#include <gtest/gtest.h>

#include <vector>

#include "libsbp_ros_msgs/geo_conversion.h"

using namespace libsbp_ros_msgs;

const double kRes = 1e-6;
const double kEnuRes = 1e-3;

TEST(GeoConversionTest, SampleGnssPoints) {
  GeographicConversion geo_conv;

  EXPECT_FALSE(geo_conv.hasEnuFrame());

  // Data from 2021_04_27_fifa/20210427094114 experiment
  Eigen::Vector3d base_pos_wgs84 = {47.3814725424, 8.57615182331,
                                    647.365379524};
  geo_conv.setEnuFrame(base_pos_wgs84.x(), base_pos_wgs84.y(),
                       base_pos_wgs84.z());
  EXPECT_TRUE(geo_conv.hasEnuFrame());
  Eigen::Vector3d base_pos_ecef_expected = {4278630.909181, 645260.577301,
                                            4671061.281798};

  // WGS84 to ECEF
  Eigen::Vector3d base_pose_ecef_calc;
  geo_conv.convertWgs84ToEcef(base_pos_wgs84, &base_pose_ecef_calc);
  EXPECT_TRUE(base_pose_ecef_calc.isApprox(base_pos_ecef_expected, kRes));

  // ECEF to WGS84
  Eigen::Vector3d base_pose_wgs84_calc;
  geo_conv.convertEcefToWgs84(base_pos_ecef_expected, &base_pose_wgs84_calc);
  EXPECT_TRUE(base_pose_wgs84_calc.isApprox(base_pos_wgs84, kRes));

  // A couple of sample points logged with Piksi CPP driver.
  // The ENU position is derived from base_pos_ned
  const std::vector<Eigen::Vector3d> enu_measured = {
      {10.863, 2.435, 1.514},
      {10.703000, 2.787000, 8.211000},
      {10.811000, 3.065000, 17.840000},
      {10.841000, 3.162000, 20.986000},
      {11.702000, -20.480000, 41.105000}};
  const std::vector<Eigen::Vector3d> wgs84_measured = {
      {47.381494, 8.576296, 648.879785},
      {47.381498, 8.576294, 655.576665},
      {47.381500, 8.576295, 665.205155},
      {47.381501, 8.576295, 668.351331},
      {47.381288, 8.576307, 688.470171}};
  const std::vector<Eigen::Vector3d> ecef_measured = {
      {4278628.531598, 645271.204977, 4671064.044718},
      {4278632.782633, 645271.684303, 4671069.211647},
      {4278639.011154, 645272.732183, 4671076.485160},
      {4278641.042840, 645273.068989, 4671078.865808},
      {4278671.587614, 645278.546191, 4671077.662780}};

  for (size_t i = 0; i < enu_measured.size(); i++) {
    // Test WGS84 <-> ENU
    Eigen::Vector3d enu_from_wgs84, wgs84_from_enu;
    // WGS84 -> ENU
    EXPECT_TRUE(geo_conv.convertWgs84ToEnu(wgs84_measured[i], &enu_from_wgs84));
    EXPECT_TRUE(enu_from_wgs84.isApprox(enu_measured[i], kEnuRes))
        << "enu_from_wgs84: " << enu_from_wgs84.transpose() << " enu_measured[" << i
        << "]: " << enu_measured[i].transpose();
    EXPECT_TRUE(geo_conv.convertEnuToWgs84(enu_from_wgs84, &wgs84_from_enu));
    EXPECT_TRUE(wgs84_from_enu.isApprox(wgs84_measured[i], kEnuRes));
    // ENU -> WGS84
    EXPECT_TRUE(geo_conv.convertEnuToWgs84(enu_measured[i], &wgs84_from_enu));
    EXPECT_TRUE(wgs84_from_enu.isApprox(wgs84_measured[i], kEnuRes));
    EXPECT_TRUE(geo_conv.convertWgs84ToEnu(wgs84_from_enu, &enu_from_wgs84));
    EXPECT_TRUE(enu_from_wgs84.isApprox(enu_measured[i], kEnuRes));

    // Test ECEF <-> ENU
    Eigen::Vector3d enu_from_ecef, ecef_from_enu;
    // ECEF -> ENU
    EXPECT_TRUE(geo_conv.convertEcefToEnu(ecef_measured[i], &enu_from_ecef));
    EXPECT_TRUE(enu_from_ecef.isApprox(enu_measured[i], kEnuRes));
    EXPECT_TRUE(geo_conv.convertEnuToEcef(enu_from_ecef, &ecef_from_enu));
    EXPECT_TRUE(ecef_from_enu.isApprox(ecef_measured[i], kEnuRes));
    // ENU -> ECEF
    EXPECT_TRUE(geo_conv.convertEnuToEcef(enu_measured[i], &ecef_from_enu));
    EXPECT_TRUE(ecef_from_enu.isApprox(ecef_measured[i], kEnuRes));
    EXPECT_TRUE(geo_conv.convertEcefToEnu(ecef_from_enu, &enu_from_ecef));
    EXPECT_TRUE(enu_from_ecef.isApprox(enu_measured[i], kEnuRes));

    // Test WGS84 <-> ECEF
    Eigen::Vector3d ecef_from_wgs84, wgs84_from_ecef;
    // WGS84 -> ECEF
    geo_conv.convertWgs84ToEcef(wgs84_measured[i], &ecef_from_wgs84);
    EXPECT_TRUE(ecef_from_wgs84.isApprox(ecef_measured[i], kRes));
    geo_conv.convertEcefToWgs84(ecef_from_wgs84, &wgs84_from_ecef);
    EXPECT_TRUE(wgs84_from_ecef.isApprox(wgs84_measured[i], kRes));
    // ECEF -> WGS84
    geo_conv.convertEcefToWgs84(ecef_measured[i], &wgs84_from_ecef);
    EXPECT_TRUE(wgs84_from_ecef.isApprox(wgs84_measured[i], kRes));
    geo_conv.convertWgs84ToEcef(wgs84_from_ecef, &ecef_from_wgs84);
    EXPECT_TRUE(ecef_from_wgs84.isApprox(ecef_measured[i], kRes));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
