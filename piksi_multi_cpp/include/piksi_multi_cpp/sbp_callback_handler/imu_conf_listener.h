#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_IMU_CONF_LISTENER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_IMU_CONF_LISTENER_H_

#include <libsbp/imu.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <memory>
#include <optional>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {

// Converts and stores the IMU configuration.
class ImuConfListener : public SBPCallbackHandler {
 public:
  ImuConfListener(const ros::NodeHandle& nh,
                  const std::shared_ptr<sbp_state_t>& state);

  bool getAccelerometerScale(double* acc_scale) const;
  bool getGyroscopeScale(double* gyro_scale) const;
  bool getImuTemp(double* temp) const;
  bool getImuType(uint8_t* type) const;

 private:
  void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) override;

  std::optional<msg_imu_aux_t> imu_aux_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_IMU_CONF_LISTENER_H_
