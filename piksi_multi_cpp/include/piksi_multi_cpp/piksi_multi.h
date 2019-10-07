#ifndef PIKSI_MULTI_CPP_PIKSI_MULTI_H_
#define PIKSI_MULTI_CPP_PIKSI_MULTI_H_

#include <libsbp/sbp.h>
#include <ros/ros.h>
#include <memory>
#include "piksi_multi_cpp/device.h"

namespace piksi_multi_cpp {

class PiksiMulti {
 public:
  PiksiMulti(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool open();
  void close();
  void read();

 private:
  void getROSParameters();
  void advertiseTopics();

  // SBP callbacks.
  static void callbackHeartbeat(uint16_t sender_id, uint8_t len, uint8_t msg[],
                                void* context);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  struct Parameters {
    Parameters() {}
  };

  Parameters params_;

  Device* current_device_;
  std::vector<std::shared_ptr<Device>> devices_;
  std::map<std::shared_ptr<Device>, sbp_state_t> states_;

  std::map<std::shared_ptr<Device>, sbp_msg_callbacks_node_t>
      heartbeat_callback_nodes_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_PIKSI_MULTI_H_
