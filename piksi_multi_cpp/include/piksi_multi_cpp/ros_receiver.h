#ifndef PIKSI_MULTI_CPP_ROS_RECEIVER_H_
#define PIKSI_MULTI_CPP_ROS_RECEIVER_H_

#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include "piksi_multi_cpp/device.h"

namespace piksi_multi_cpp {

// This class offers a ROS interface for Piksi Multi receivers. It automatically
// assigns a type to a device and offers ROS topics and services.
class ROSReceiver {
 public:
  /* The three types of receivers are

  kBaseStation: The static base station sending out RTK corrections.

  kPositionReceiver: The moving rover receiving RTK corrections from the
  base station and broadcasting RTK GPS positions.

  kAttitudeReceiver: The moving rover receiving RTK corrections from a moving
  reference receiver and broadcasting the moving baseline (also referred to as
  heading). */
  enum Type {
    kBaseStation = 0,
    kPositionReceiver,
    kAttitudeReceiver,
    kUnknown
  };
  static std::vector<Type> kTypeVec;

  ROSReceiver(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
              const std::shared_ptr<Device>& device, const std::string& ns);

  // Closes device.
  ~ROSReceiver();

  // Open device.
  bool init();
  // Read device and process SBP callbacks.
  void process();

  // Factory method to create all receivers.
  static std::shared_ptr<ROSReceiver> create(
      const Type type, const ros::NodeHandle& nh,
      const ros::NodeHandle& nh_private, const std::shared_ptr<Device>& device,
      const std::string& ns);
  static std::vector<std::shared_ptr<ROSReceiver>> createAllReceivers(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 protected:
  // The actual hardware interface.
  std::shared_ptr<Device> device_;

  // The receiver namespace inferred from the type and unique_id.
  std::string ns_;

 private:
  // Infer receiver type from Piksi firmware settings.
  static Type inferType(const std::shared_ptr<Device>& dev);
  static std::string createNameSpace(const Type type, const size_t id);

  // ROS publishers common for every type.
  ros::Publisher heartbeat_pub_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_ROS_RECEIVER_H_
