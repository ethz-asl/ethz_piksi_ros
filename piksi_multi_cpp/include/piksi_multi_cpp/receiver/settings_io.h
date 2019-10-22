#ifndef PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_
#define PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_

#include "piksi_multi_cpp/receiver/receiver.h"

#include <libsbp/settings.h>
#include <memory>
#include <string>

namespace piksi_multi_cpp {

class SettingsIo : public Receiver {
 public:
  typedef std::shared_ptr<SettingsIo> Ptr;
  SettingsIo(const ros::NodeHandle& nh, const Device::Ptr& device);

  // Interface to read settings from device.
  bool readSetting(const std::string& section, const std::string& name,
                   const int timeout_ms = 1000);
  // Interface to write settings to device.
  bool writeSetting(const std::string& section, const std::string& name,
                    const std::string& value, const int timeout_ms = 1000);

  // Compare the stored value with a string.
  bool compareValue(const std::string& value) const;
  // Return the recently read setting.
  std::string getValue() const;
  // Check whether the value is a true boolean.
  bool checkBoolTrue() const;
  // Open and parse a piksi config .ini file.
  bool openConfig(const std::string& file);

 private:
  void receiveReadResponse(const msg_settings_read_resp_t& msg,
                           const uint8_t len);
  void receiveWriteResponse(const msg_settings_write_resp_t& msg,
                            const uint8_t len);

  std::string value_;
  bool write_success_{false};
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_
