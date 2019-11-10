#include <eigen_conversions/eigen_msg.h>
#include <geotf/geodetic_converter.h>
#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <piksi_rtk_msgs/SamplePosition.h>
#include <boost/algorithm/string.hpp>
#include "piksi_multi_cpp/receiver/receiver_base_station.h"

namespace piksi_multi_cpp {

ReceiverBaseStation::ReceiverBaseStation(const ros::NodeHandle& nh,
                                         const Device::Ptr& device)
    : ReceiverRos(nh, device) {
  setupUDPSenders();
  setupBaseStationSampling();
}

void ReceiverBaseStation::setupBaseStationSampling() {
  // Subscribe to maximum likelihood estimate and advertise service to overwrite
  // current base station position.
  overwrite_base_position_srv_ = nh_.advertiseService(
      "overwrite_base_position",
      &ReceiverBaseStation::overwriteBasePositionCallback, this);
  const uint32_t kQueueSizeMlEstimate = 0;
  ml_estimate_sub_ =
      nh_.subscribe("position_sampler/ml_position", kQueueSizeMlEstimate,
                    &ReceiverBaseStation::sampledPositionCallback, this);

  // Automatically sample base station on startup.
  ros::NodeHandle nh_node("~");
  auto autostart_base_sampling =
      nh_node.param<bool>("autostart_base_sampling", true);
  if (autostart_base_sampling) {
    std_srvs::Empty srv;
    overwriteBasePositionCallback(srv.request, srv.response);
  }
}

void ReceiverBaseStation::setupUDPSenders() {
  auto udp_port = nh_.param<int>("udp_port", 26078);

  std::vector<std::string> udp_interfaces =
      getVectorParam("udp_broadcast_interfaces");
  for (const auto& interface : udp_interfaces) {
    auto udp_sender_ = UDPObservationSender::toNetwork(interface, udp_port);
    if (udp_sender_) {
      udp_sender_->open();
      // Register with observation callbacks
      obs_cbs_->addObservationCallbackListener(
          CBtoRawObsConverter::createFor(udp_sender_));
    }
  }

  // set up unicasts
  std::vector<std::string> udp_unicasts = getVectorParam("udp_unicast_targets");

  for (const auto& unicast : udp_unicasts) {
    auto udp_sender_ = UDPObservationSender::toHost(unicast, udp_port);
    if (udp_sender_) {
      udp_sender_->open();

      // Register with observation callbacks
      obs_cbs_->addObservationCallbackListener(
          CBtoRawObsConverter::createFor(udp_sender_));
    }
  }
}

bool ReceiverBaseStation::overwriteBasePositionCallback(
    std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  // Set flag to wait for sampling to be finished.
  wait_for_sampled_position_ = true;
  // Start sampling.
  ros::NodeHandle nh_node("~");
  uint32_t num_desired_fixes = nh_node.param<int>("num_desired_fixes", 2000);
  position_sampler_->startSampling(num_desired_fixes);
  ROS_INFO("Start sampling base station position with %d desired fixes.",
           num_desired_fixes);
  return true;
}

void ReceiverBaseStation::sampledPositionCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::Ptr& msg) {
  if (!wait_for_sampled_position_) {
    ROS_WARN("Received sampled base position but not updating firmware.");
    ROS_WARN("Call `overwrite_base_position` first.");
    return;
  }
  wait_for_sampled_position_ = false;

  Eigen::Vector3d x_ecef, x_wgs84;
  tf::pointMsgToEigen(msg->position.position, x_ecef);
  geotf::GeodeticConverter geotf;
  geotf.addFrameByEPSG("ecef", 4978);
  geotf.addFrameByEPSG("wgs84", 4326);

  if (!geotf.convert("ecef", x_ecef, "wgs84", &x_wgs84)) {
    ROS_ERROR("Failed to convert ECEF to WGS84.");
    return;
  }

  ROS_INFO("Writing lat: %.9f lon: %.9f alt: %.9f to %s.", x_wgs84.x(),
           x_wgs84.y(), x_wgs84.z(), nh_.getUnresolvedNamespace().c_str());

  ROS_ERROR_COND(!writeSetting("surveyed_position", "surveyed_lat",
                               boost::lexical_cast<std::string>(x_wgs84.x())),
                 "Failed to overwrite surveyed_lat.");

  ROS_ERROR_COND(!writeSetting("surveyed_position", "surveyed_lon",
                               boost::lexical_cast<std::string>(x_wgs84.y())),
                 "Failed to overwrite surveyed_lon.");

  ROS_ERROR_COND(!writeSetting("surveyed_position", "surveyed_alt",
                               boost::lexical_cast<std::string>(x_wgs84.z())),
                 "Failed to overwrite surveyed_alt.");
}

}  // namespace piksi_multi_cpp
