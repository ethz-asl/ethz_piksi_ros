#ifndef PIKSI_MULTI_CPP_PIKSI_MULTI_H_
#define PIKSI_MULTI_CPP_PIKSI_MULTI_H_

#include <ros/ros.h>

namespace piksi_multi_cpp {

class PiksiMulti {
public:
  PiksiMulti(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  bool connect();
  bool disconnect();
  bool read();

private:
  void getROSParameters();
  void advertiseTopics();

  bool connectUSB();
  bool connectSerial();
  bool connectTCP();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  struct Parameters {

    Parameters() {}

    std::string interface = "usb";
    std::string port = "/dev/ttyACM0";
  };

  Parameters params_;
  enum class Interface { kUSB = 0, kSerial, kTCP, kInvalid };
  const std::map<std::string, Interface> kStrToInterface = {
      {"usb", Interface::kUSB},
      {"serial", Interface::kSerial},
      {"tcp", Interface::kTCP}};
};

} // namespace piksi_multi_cpp

#endif // PIKSI_MULTI_CPP_PIKSI_MULTI_H_
