#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_UTC_TIME_BUFFER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_UTC_TIME_BUFFER_H_

#include <libsbp/sbp.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <memory>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {

// Singleton class to buffer the last GPS time messages. Used to convert tow,
// tow_f to more precise GPS timestamp.
// TODO(rikba): Make singleton
class UtcTimeBuffer : public SBPCallbackHandler {
 public:
  UtcTimeBuffer(const ros::NodeHandle& nh,
                const std::shared_ptr<sbp_state_t>& state);

  ros::Time getTime(const uint32_t tow);

 private:
  void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) override;

  std::map<uint32_t, ros::Time> time_map_;
  size_t buffer_size_ = 100;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_UTC_TIME_BUFFER_H_
