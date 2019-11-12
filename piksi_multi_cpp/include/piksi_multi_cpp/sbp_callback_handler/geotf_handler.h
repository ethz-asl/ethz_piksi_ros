#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_GEOTF_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_GEOTF_HANDLER_H_

#include <geotf/geodetic_converter.h>
#include <libsbp/navigation.h>
#include <Eigen/Dense>
#include <memory>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h"

namespace piksi_multi_cpp {

// Manages a geotf object to transform between frames.
class GeoTfHandler {
 public:
  typedef std::shared_ptr<GeoTfHandler> Ptr;
  GeoTfHandler(const std::shared_ptr<sbp_state_t>& state);

  void setEnuOriginWgs84(const double lat, const double lon, const double alt);
  void resetEnuOrigin();

  bool convertPosEcefToEnu(const Eigen::Vector3d& pos_ecef,
                           const Eigen::Vector3d* pos_enu);

  GeoTfHandler(GeoTfHandler const&) = delete;
  void operator=(GeoTfHandler const&) = delete;

 private:
  void callbackToPosLlh(const msg_pos_llh_t& msg, const uint8_t len);

  bool use_base_enu_origin_ = true;  // False: ENU origin is first position.
  SBPLambdaCallbackHandler<msg_pos_llh_t> pos_llh_handler_;
  geotf::GeodeticConverter geotf_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_GEOTF_HANDLER_H_
