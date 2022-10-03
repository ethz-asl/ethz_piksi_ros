#include <eigen_conversions/eigen_msg.h>
#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <piksi_rtk_msgs/SamplePosition.h>
#include <boost/algorithm/string.hpp>
#include <string>
#include "piksi_multi_cpp/receiver/receiver_base_station.h"

namespace piksi_multi_cpp {

ReceiverBaseStation::ReceiverBaseStation(const ros::NodeHandle& nh,
                                         const Device::Ptr& device)
    : ReceiverRos(nh, device) {
  setupBaseStationSampling();
}

bool ReceiverBaseStation::init() {
  // Init base class.
  if (!ReceiverRos::init()) {
    return false;
  }

  // Setup UDP senders.
  while (!readSetting("system_info", "sbp_sender_id")) {
  }
  sbp_sender_id_ = static_cast<uint16_t>(std::stoul(getValue(), nullptr, 16));
  ROS_INFO("UDP corrections sender ID: 0x%.4X", sbp_sender_id_);
  setupUDPSenders();

  return true;
}

// The base station can either be sampled automatically at startup with
// `autostart_base_sampling` rosparam or with a service call
// `resample_base_position`.
// The number of desired fixes is determined through the parameter
// `num_desired_fixes` when autosampling or defined in the service call.
void ReceiverBaseStation::setupBaseStationSampling() {
  // Subscribe to maximum likelihood estimate and advertise service to overwrite
  // current base station position.
  resample_base_position_srv_ = nh_.advertiseService(
      "resample_base_position",
      &ReceiverBaseStation::resampleBasePositionCallback, this);
  const uint32_t kQueueSizeMlEstimate = 0;
  ml_estimate_sub_ =
      nh_.subscribe("position_sampler/ml_position", kQueueSizeMlEstimate,
                    &ReceiverBaseStation::sampledPositionCallback, this);

  // Automatically sample base station on startup.
  ros::NodeHandle nh_node("~");
  auto autostart_base_sampling =
      nh_node.param<bool>("autostart_base_sampling", true);
  if (autostart_base_sampling) {
    piksi_rtk_msgs::SamplePosition srv;
    srv.request.num_desired_fixes =
        nh_node.param<int>("num_desired_fixes", 2000);
    resampleBasePositionCallback(srv.request, srv.response);
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
          CBtoRawObsConverter::createFor(udp_sender_, sbp_sender_id_));
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
          CBtoRawObsConverter::createFor(udp_sender_, sbp_sender_id_));
    }
  }
}

bool ReceiverBaseStation::resampleBasePositionCallback(
    piksi_rtk_msgs::SamplePosition::Request& req,
    piksi_rtk_msgs::SamplePosition::Response& res) {
  if (!position_sampler_->startSampling(req.num_desired_fixes, req.file,
                                        req.set_enu, req.offset_z)) {
    return false;
  }

  // Set flag to wait for sampling to be finished.
  wait_for_sampled_position_ = true;
  return true;
}

void ReceiverBaseStation::sampledPositionCallback(
    const piksi_rtk_msgs::PositionWithCovarianceStamped::Ptr& msg) {
  if (!wait_for_sampled_position_) {
    ROS_WARN("Received sampled base position but not saving to settings.");
    ROS_WARN("Call `resample_base_position` to resample base station.");
    return;
  }
  wait_for_sampled_position_ = false;

  if (!geotf_handler_.get()) {
    ROS_ERROR("Geotf not set.");
    return;
  }

  Eigen::Vector3d x_ecef, x_wgs84;
  tf::pointMsgToEigen(msg->position.position, x_ecef);
  geotf_handler_->getGeoTf().convertEcefToWgs84(x_ecef, &x_wgs84);

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
