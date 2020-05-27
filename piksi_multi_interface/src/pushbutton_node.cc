#include <ros/ros.h>

#include <gpiod.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_pushbutton");
  ros::NodeHandle nh("~");

  // ROS parameters
  int rate = 1000;
  nh.getParam("rate", rate);

  std::string chip = "gpiochip0";
  nh.getParam("chip", chip);

  int offset = 0;
  nh.getParam("offset", offset);

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

    ros::spinOnce();
    loop_rate.sleep();
  }

  gpiod_line_close_chip(line);
  return 0;
}
