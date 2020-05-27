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

  int line = 0;
  nh.getParam("line", line);

  ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("status", 1);

  ros::Rate loop_rate(rate);
  while (ros::ok()) {
    std_msgs::Bool status;
    auto ret = gpiod_ctxless_get_value(chip.c_str(), line, false,
                                       ros::this_node::getName().c_str());
    if (ret < 0) {
      ROS_ERROR("Cannot get GPIO value on chip %s line %d", chip.c_str(), line);
      ros::spinOnce();
      return 0;
    }

    status.data = ret;
    status_pub.publish(status);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
