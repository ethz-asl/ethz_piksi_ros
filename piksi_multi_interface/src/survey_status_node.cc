#include <geometry_msgs/PointStamped.h>
#include <gpiod.h>
#include <piksi_rtk_msgs/PositionSampling.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <optional>

Eigen::Vector3d eigenFromMsg(const geometry_msgs::PointStampedConstPtr& msg) {
  Eigen::Vector3d pos;
  tf2::fromMsg(msg->point, pos);
  return pos;
}

/*
 * Compares broadcasted base station position to current position to check if
 * surveyed position is valid. Checks position sampling feedback to blink.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "survey_status");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // ROS parameters
  int rate = 2;
  nh_private.getParam("rate", rate);

  double timeout = 1.0;
  nh_private.getParam("timeout", timeout);

  double distance_threshold = 1.0;
  nh_private.getParam("distance_threshold", distance_threshold);

  std::string chip = "gpiochip0";
  nh_private.getParam("chip", chip);

  int offset = 0;
  nh_private.getParam("offset", offset);

  // ROS subscribers
  std::optional<Eigen::Vector3d> current_pos, sampled_pos;
  ros::Subscriber current_sub = nh.subscribe<geometry_msgs::PointStamped>(
      "/piksi_multi_cpp_base/base_station_receiver_0/ros/pos_ecef", 1,
      [&](const geometry_msgs::PointStampedConstPtr& msg) {
        current_pos = eigenFromMsg(msg);
      });
  ros::Subscriber sampled_sub = nh.subscribe<geometry_msgs::PointStamped>(
      "/piksi_multi_cpp_base/base_station_receiver_0/ros/base_pos_ecef", 1,
      [&](const geometry_msgs::PointStampedConstPtr& msg) {
        sampled_pos = eigenFromMsg(msg);
      });

  // Open GPIO
  auto gpio = gpiod_chip_open_by_name(chip.c_str());
  ROS_INFO_COND(gpio, "Opened GPIO chip %s.", chip.c_str());
  if (!gpio) {
    ROS_ERROR("Cannot open GPIO %s.", chip.c_str());
    ros::spinOnce();
    return 0;
  }

  // Get Line.
  auto line = gpiod_chip_get_line(gpio, offset);
  ROS_INFO_COND(line, "Opened line chip %d.", offset);
  if (!gpio) {
    ROS_ERROR("Cannot open line %d on chip %s.", offset, chip.c_str());
    gpiod_chip_close(gpio);
    ros::spinOnce();
    return 0;
  }

  // Configure GPIO.
  if (gpiod_line_request_output(line, ros::this_node::getName().c_str(), 0) <
      0) {
    ROS_ERROR("Cannot request output GPIO on chip %s line %d", chip.c_str(),
              offset);
    gpiod_line_close_chip(line);
    ros::spinOnce();
    return 0;
  }
  ROS_INFO("Configured GPIO as output.");

  ros::Rate loop_rate(rate);
  while (ros::ok()) {
    // Check if sampling is currently ongoing.
    auto sampling_feedback =
        ros::topic::waitForMessage<piksi_rtk_msgs::PositionSampling>(
            "/piksi_multi_cpp_base/base_station_receiver_0/position_sampler/"
            "position_sampling",
            nh, ros::Duration(timeout));
    if (sampling_feedback && (sampling_feedback->progress > -1) &&
        (sampling_feedback->progress < 100)) {
      // If sampling, toggle LED.
      auto ret = gpiod_line_get_value(line);
      if (ret < 0) {
        ROS_ERROR("Cannot get GPIO value on chip %s line %d", chip.c_str(),
                  offset);
        gpiod_line_close_chip(line);
        ros::spinOnce();
        return 0;
      }

      ret = gpiod_line_set_value(line, ret == 0 ? 1 : 0);
      if (ret < 0) {
        ROS_ERROR("Cannot set GPIO value on chip %s line %d", chip.c_str(),
                  offset);
        gpiod_line_close_chip(line);
        ros::spinOnce();
        return 0;
      }
    } else if (sampled_pos.has_value() && current_pos.has_value()) {
      // Compare stored base station position to current GNSS position.
      auto distance = (current_pos.value() - sampled_pos.value()).norm();

      if (gpiod_line_set_value(line, distance < distance_threshold ? 1 : 0) <
          0) {
        ROS_ERROR("Cannot set GPIO value on chip %s line %d", chip.c_str(),
                  offset);
        gpiod_line_close_chip(line);
        ros::spinOnce();
        return 0;
      }

      if (distance > distance_threshold) {
        ROS_WARN_THROTTLE(10.0,
                          "Distance between sampled position and current GNSS "
                          "position is greater than threshold: %.2f > %.2f. Is "
                          "the base station sampled correctly?",
                          distance, distance_threshold);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Turn off LED.
  if (gpiod_line_set_value(line, 0) < 0) {
    ROS_ERROR("Cannot set GPIO value on chip %s line %d", chip.c_str(), offset);
  }

  // Release LED.
  gpiod_line_close_chip(line);
  return 0;
}
