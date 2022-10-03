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
      {10.857000000000001, 2.353, 0.198},
      {10.036, -17.792, 41.247},
      {6.317, 5.981, 41.232},
      {2.852, 39.489000000000004, 41.238}};
  const std::vector<Eigen::Vector3d> wgs84_measured = {
      {47.38149370420578, 8.57629557889748, 647.5638267526044},
      {47.38131252854044, 8.576284714398815, 688.6127781794771},
      {47.38152633220841, 8.576235472927848, 688.5978165181814},
      {47.381827687678694, 8.576189587067374, 688.6035694238228}};
  const std::vector<Eigen::Vector3d> ecef_measured = {
      {4278627.710932162, 645271.074246071, 4671063.021060776},
      {4278669.975769749, 645276.6185371446, 4671079.587635878},
      {4278653.221991419, 645270.3310393207, 4671095.673600099},
      {4278629.360581043, 645263.227948546, 4671118.366348593}};

  for (size_t i = 0; i < enu_measured.size(); i++) {
    // Test WGS84 <-> ENU
    Eigen::Vector3d enu_from_wgs84, wgs84_from_enu;
    // WGS84 -> ENU
    EXPECT_TRUE(geo_conv.convertWgs84ToEnu(wgs84_measured[i], &enu_from_wgs84));
    EXPECT_TRUE(enu_from_wgs84.isApprox(enu_measured[i], kEnuRes))
        << "wgs84_measured[" << i << "]: " << wgs84_measured[i].transpose()
        << " enu_from_wgs84: " << enu_from_wgs84.transpose() << " enu_measured["
        << i << "]: " << enu_measured[i].transpose();
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
