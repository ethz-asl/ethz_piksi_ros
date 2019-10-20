#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_TIME_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_TIME_HANDLER_H_

#include <libsbp/navigation.h>
#include <libsbp/sbp.h>
#include <ros/time.h>
#include <memory>
#include <optional>

#include "piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h"

namespace piksi_multi_cpp {

// This class allows converting tow information in ROS UTC time. For this
// purpose it listens to the GPS time and UTC time which is emitted by the
// device. In the simplest case one can just lookup the UTC time given the tow
// of the measurement. In the more complicated case the time needs to be
// converted from GPS time to UTC time. To class determines the leap second
// offset from two consecutive GPS and UTC messages.
class RosTimeHandler {
 public:
  typedef std::shared_ptr<RosTimeHandler> Ptr;
  RosTimeHandler(const std::shared_ptr<sbp_state_t>& state);
  RosTimeHandler(RosTimeHandler const&) = delete;
  void operator=(RosTimeHandler const&) = delete;

  ros::Time lookupTime(const uint32_t tow);
  ros::Time lookupTime(const uint32_t tow, const uint8_t tow_f);

 private:
  void callbackToGpsTime(const msg_gps_time_t& msg);
  void callbackToUtcTime(const msg_utc_time_t& msg);

  bool use_gps_time_ = false;
  SBPLambdaCallbackHandler<msg_gps_time_t> gps_time_handler_;
  SBPLambdaCallbackHandler<msg_utc_time_t> utc_time_handler_;
  std::optional<ros::Time> last_gps_time_;
  std::optional<uint32_t> leap_seconds_;
  std::optional<std::pair<uint32_t, ros::Time>> tow_to_utc_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_TIME_HANDLER_H_
