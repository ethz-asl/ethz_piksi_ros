#include "piksi_multi_cpp/settings_io/settings_io.h"

#include <libsbp/settings.h>
#include <ros/console.h>
#include <functional>
#include <memory>

namespace piksi_multi_cpp {

SettingsIo::SettingsIo(const Device::Ptr& device)
    : device_(device),
      state_(std::make_shared<sbp_state_t>()),
      settings_listener_{
          std::bind(&SettingsIo::printSetting, this, std::placeholders::_1),
          SBP_MSG_SETTINGS_READ_RESP, state_} {
  sbp_state_init(state_.get());
  sbp_state_set_io_context(state_.get(), this);
}

bool SettingsIo::readSetting(const std::string& section,
                             const std::string& name) {
  const uint16_t kSbpSenderId = 0x43;  // Some arbitrary number.

  // Send MSG_SETTINGS_READ_REQ
  std::string setting = section + "\0" + name + "\0";  // Parse setting name.
  msg_settings_read_req_t read_req;
  read_req.setting[0] = *setting.c_str();
  ROS_INFO("Requesting setting: %s", read_req.setting);
  sbp_send_message(state_.get(), SBP_MSG_SETTINGS_READ_REQ, kSbpSenderId,
                   sizeof(read_req), reinterpret_cast<uint8_t*>(&read_req),
                   &SettingsIo::write_redirect);

  // Wait for  MSG_SETTINGS_READ_RESP.

  return true;
}

void SettingsIo::printSetting(const msg_settings_read_resp_t& msg) {
  ROS_INFO("Received setting: %s", msg.setting);
}

int32_t SettingsIo::write_redirect(uint8_t* buff, uint32_t n, void* context) {
  if (!context) {
    ROS_ERROR_STREAM("No context set.");
    return 0;
  }
  // Cast context to instance.
  SettingsIo* instance = static_cast<SettingsIo*>(context);

  if (!instance->device_.get()) {
    ROS_ERROR_STREAM("No device set.");
    return 0;
  }

  // Execute instance's write function.
  return instance->device_->write(std::vector<uint8_t>(&buff[0], &buff[n]));
}

}  // namespace piksi_multi_cpp
