#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_GEOTF_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_GEOTF_HANDLER_H_

#include <geotf/geodetic_converter.h>
#include <libsbp/navigation.h>
#include <piksi_rtk_msgs/EnuOrigin.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>
#include <memory>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h"

namespace piksi_multi_cpp {

// Manages a geotf object to transform between frames.
// The ENU frame is either set automatically to be the base station position,
// through ROS parameters or reset through a service call.
class GeoTfHandler {
 public:
  typedef std::shared_ptr<GeoTfHandler> Ptr;
  GeoTfHandler(const ros::NodeHandle& nh,
               const std::shared_ptr<sbp_state_t>& state);

  void setEnuOriginWgs84(const double lat, const double lon, const double alt);

  bool convertPosEcefToEnu(const Eigen::Vector3d& pos_ecef,
                           const Eigen::Vector3d* pos_enu);

  inline geotf::GeodeticConverter getGeoTf() const { return geotf_; }

  GeoTfHandler(GeoTfHandler const&) = delete;
  void operator=(GeoTfHandler const&) = delete;

 private:
  void callbackToBasePosEcef(const msg_base_pos_ecef_t& msg, const uint8_t len);
  void callbackToPosLlh(const msg_pos_llh_t& msg, const uint8_t len);
  bool setEnuOriginCallback(piksi_rtk_msgs::EnuOrigin::Request& req,
                            piksi_rtk_msgs::EnuOrigin::Response& res);
  bool setEnuOriginFromBaseStation(std_srvs::Empty::Request& req,
                                   std_srvs::Empty::Response& res);
  bool setEnuOriginFromCurrentPos(std_srvs::Empty::Request& req,
                                  std_srvs::Empty::Response& res);

  SBPLambdaCallbackHandler<msg_base_pos_ecef_t> base_pos_ecef_handler_;
  SBPLambdaCallbackHandler<msg_pos_llh_t> pos_llh_handler_;
  geotf::GeodeticConverter geotf_;
  Eigen::Vector3d enu_origin_wgs84_;

  ros::NodeHandle nh_;
  ros::ServiceServer set_enu_origin_srv_;
  ros::ServiceServer set_enu_from_base_srv_;
  ros::ServiceServer set_enu_from_current_srv_;

  enum ResetEnuOrigin { kNo = 0, kFromBase, kFromCurrentPos };
  ResetEnuOrigin reset_position_ = ResetEnuOrigin::kFromCurrentPos;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_GEOTF_HANDLER_H_
