#include "piksi_multi_cpp/ros_output_heartbeat.h"

#include <libsbp/system.h>
#include <piksi_rtk_msgs/Heartbeat.h>

namespace piksi_multi_cpp {

ROSOutputHeartbeat::ROSOutputHeartbeat(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state)
    : ROSOutput(nh, sbp_msg_type, state) {
  // Advertise ROS topics.
  relay_pub_ = nh_.advertise<piksi_rtk_msgs::Heartbeat>("heartbeat", kQueueSize,
                                                        kLatchTopic);
}

void ROSOutputHeartbeat::callback(uint16_t sender_id, uint8_t len,
                                  uint8_t msg[]) {
  // Before doing anything check if anybody is listening.
  // https://answers.ros.org/question/197878/how-expensive-is-getnumsubscribers-of-publisher/

  // Cast message.
  auto sbp_msg_heartbeat = (msg_heartbeat_t*)msg;
  if (!sbp_msg_heartbeat) return;

  // Translate to ROS message.
  piksi_rtk_msgs::Heartbeat ros_msg_heartbeat;
  ros_msg_heartbeat.header.stamp = ros::Time::now();

  // How to shift and mask bits: https://stackoverflow.com/a/27592777
  ros_msg_heartbeat.system_error = (sbp_msg_heartbeat->flags >> 0) & 0x1;
  ros_msg_heartbeat.io_error = (sbp_msg_heartbeat->flags >> 1) & 0x1;
  ros_msg_heartbeat.swift_nap_error = (sbp_msg_heartbeat->flags >> 2) & 0x1;
  ros_msg_heartbeat.sbp_minor_version = (sbp_msg_heartbeat->flags >> 8) & 0xFF;
  ros_msg_heartbeat.sbp_major_version = (sbp_msg_heartbeat->flags >> 16) & 0xFF;
  ros_msg_heartbeat.external_antenna_present =
      (sbp_msg_heartbeat->flags >> 31) & 0x1;

  // Publish over ROS network.
  relay_pub_.publish(ros_msg_heartbeat);
}

}  // namespace piksi_multi_cpp
