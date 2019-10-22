#ifndef PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_
#define PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_

#include "piksi_multi_cpp/receiver/receiver.h"

#include <libsbp/settings.h>
#include <string>

namespace piksi_multi_cpp {

class SettingsIo : public Receiver {
 public:
  SettingsIo(const ros::NodeHandle& nh, const Device::Ptr& device);

  // Interface to read and write settings to device.
  bool readSetting(const std::string& section, const std::string& name,
                   const int timeout_ms = 1000);

  // Compare the stored value with a string.
  bool compareValue(const std::string& value) const;
  // Check whether the value is a true boolean.
  bool checkBoolTrue() const;

 private:
  void storeSetting(const msg_settings_read_resp_t& msg, const uint8_t len);

  std::string value_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_
