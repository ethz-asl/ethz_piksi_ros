#include <ros/ros.h>

#include <gpiod.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_pushbutton");
  ros::NodeHandle nh("~");

  int rate = 1000;
  nh.getParam("rate", rate);

  ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("status", 1);

  ros::Rate loop_rate(rate);
  while (ros::ok()) {
    std_msgs::Bool status;
    status.data = false;
    status_pub.publish(status);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
