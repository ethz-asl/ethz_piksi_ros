#include <ros/ros.h>

#include <gpiod.h>
#include <piksi_rtk_msgs/SamplePosition.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_pushbutton");
  ros::NodeHandle nh("~");

  // ROS parameters
  int rate = 20;
  nh.getParam("rate", rate);

  std::string chip = "gpiochip0";
  nh.getParam("chip", chip);

  int offset = 0;
  nh.getParam("offset", offset);

  std::string piksi_ns = "/piksi_multi_cpp_base/base_station_receiver_0";
  nh.getParam("piksi_ns", piksi_ns);

  // Setup sample survey call.
  bool is_base = piksi_ns.find("base_station_receiver") != std::string::npos;
  std::string service = is_base ? piksi_ns + "/resample_base_position"
                                : piksi_ns + "/sample_position";
  ROS_INFO("Pushbutton connected to %s", service.c_str());
  ros::ServiceClient client =
      nh.serviceClient<piksi_rtk_msgs::SamplePosition>(service);

  int num_desired_fixes = is_base ? 1000 : 100;
  nh.getParam("num_desired_fixes", num_desired_fixes);

  ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("status", 1);

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
  if (gpiod_line_request_input(line, ros::this_node::getName().c_str()) < 0) {
    ROS_ERROR("Cannot request input GPIO on chip %s line %d", chip.c_str(),
              offset);
    gpiod_line_close_chip(line);
    ros::spinOnce();
    return 0;
  }
  ROS_INFO("Configured GPIO as input.");

  ros::Rate loop_rate(rate);
  while (ros::ok()) {
    std_msgs::Bool status;
    auto ret = gpiod_line_get_value(line);
    if (ret < 0) {
      ROS_ERROR("Cannot get GPIO value on chip %s line %d", chip.c_str(),
                offset);
      gpiod_line_close_chip(line);
      ros::spinOnce();
      return 0;
    }

    status.data = ret;
    status_pub.publish(status);

    // Service call logic.
    if (status.data) {
      piksi_rtk_msgs::SamplePosition srv;
      srv.request.num_desired_fixes = num_desired_fixes;
      srv.request.set_enu = is_base;
      ROS_INFO_COND(client.call(srv), "Start sampling.");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  gpiod_line_close_chip(line);
  return 0;
}
