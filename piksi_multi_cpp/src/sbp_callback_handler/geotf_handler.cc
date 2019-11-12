#include "piksi_multi_cpp/sbp_callback_handler/geotf_handler.h"

#include <functional>

namespace piksi_multi_cpp {
namespace s = std::placeholders;

GeoTfHandler::GeoTfHandler(const ros::NodeHandle& nh,
                           const std::shared_ptr<sbp_state_t>& state)
    : base_pos_llh_handler_{std::bind(&GeoTfHandler::callbackToBasePosLlh, this,
                                      s::_1, s::_2),
                            SBP_MSG_BASE_POS_LLH, state},
      nh_(nh) {
  geotf_.initFromRosParam();
  geotf_.addFrameByEPSG("ecef", 4978);
  geotf_.addFrameByEPSG("wgs84", 4326);

  set_enu_origin_srv_ = nh_.advertiseService(
      "set_enu_origin", &GeoTfHandler::setEnuOriginCallback, this);
  reset_enu_origin_srv_ = nh_.advertiseService(
      "reset_enu_origin", &GeoTfHandler::resetEnuOriginCallback, this);
}

void GeoTfHandler::setEnuOriginWgs84(const double lat, const double lon,
                                     const double alt) {
  resetEnuOrigin();
  geotf_.addFrameByENUOrigin("enu", lat, lon, alt);
}

void GeoTfHandler::resetEnuOrigin() { geotf_.removeFrame("enu"); }

void GeoTfHandler::callbackToBasePosLlh(const msg_base_pos_llh_t& msg,
                                        const uint8_t len) {
  if (geotf_.hasFrame("enu")) return;  // ENU origin already set.
  geotf_.addFrameByENUOrigin("enu", msg.lat, msg.lon, msg.height);
}

bool GeoTfHandler::setEnuOriginCallback(
    piksi_rtk_msgs::EnuOrigin::Request& req,
    piksi_rtk_msgs::EnuOrigin::Response& res) {
  setEnuOriginWgs84(req.lat, req.lon, req.alt);
  return true;
}

bool GeoTfHandler::resetEnuOriginCallback(std_srvs::Empty::Request& req,
                                          std_srvs::Empty::Response& res) {
  resetEnuOrigin();
  return true;
}

}  // namespace piksi_multi_cpp
