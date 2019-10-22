#include "piksi_multi_cpp/receiver/settings_io.h"

#include <libsettings/settings_util.h>
#include <ros/assert.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h"

namespace piksi_multi_cpp {

SettingsIo::SettingsIo(const ros::NodeHandle& nh, const Device::Ptr& device)
    : Receiver(nh, device) {}

bool SettingsIo::readSetting(const std::string& section,
                             const std::string& name, const int timeout_ms) {
  value_.clear();

  // Parse request to format setting\0name\0
  size_t kLen = section.size() + name.size() + 2;
  char read_req[kLen] = {0};
  settings_format(section.c_str(), name.c_str(), nullptr, nullptr, read_req,
                  kLen);

  // Register setting listener.
  SBPLambdaCallbackHandler<msg_settings_read_resp_t> settings_listener(
      std::bind(&SettingsIo::receiveReadResponse, this, std::placeholders::_1,
                std::placeholders::_2),
      SBP_MSG_SETTINGS_READ_RESP, state_);

  // Send message.
  int req_success = sbp_send_message(
      state_.get(), SBP_MSG_SETTINGS_READ_REQ, SBP_SENDER_ID, kLen,
      (unsigned char*)(&read_req), &Device::write_redirect);
  if (req_success != SBP_OK) {
    ROS_ERROR("Cannot request setting %s.%s, %d", section.c_str(), name.c_str(),
              req_success);
    return false;
  }

  // Wait to receive setting.
  if (!settings_listener.waitForCallback(timeout_ms)) {
    ROS_ERROR("Did not receive setting %s.%s", section.c_str(), name.c_str());
    return false;
  }

  return true;
}

bool SettingsIo::writeSetting(const std::string& section,
                              const std::string& name, const std::string& value,
                              const int timeout_ms) {
  // Parse request to format setting\0name\0value\0
  size_t kLen = section.size() + name.size() + value.size() + 2;
  char write_req[kLen] = {0};
  settings_format(section.c_str(), name.c_str(), value.c_str(), nullptr,
                  write_req, kLen);

  // Register setting listener.
  SBPLambdaCallbackHandler<msg_settings_write_resp_t> write_resp_listener(
      std::bind(&SettingsIo::receiveWriteResponse, this, std::placeholders::_1,
                std::placeholders::_2),
      SBP_MSG_SETTINGS_WRITE_RESP, state_);

  // Send message.
  int req_success = sbp_send_message(
      state_.get(), SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, kLen,
      (unsigned char*)(&write_req), &Device::write_redirect);
  if (req_success != SBP_OK) {
    ROS_ERROR("Cannot write setting %s.%s.%s, %d", section.c_str(),
              name.c_str(), value.c_str(), req_success);
    return false;
  }

  // Wait to receive setting.
  if (!write_resp_listener.waitForCallback(timeout_ms)) {
    ROS_ERROR("Did not receive write response %s.%s.%s", section.c_str(),
              name.c_str(), value.c_str());
    return false;
  }

  return true;
}

void SettingsIo::receiveReadResponse(const msg_settings_read_resp_t& msg,
                                     const uint8_t len) {
  // Parse settings.
  const char *section = nullptr, *name = nullptr, *value = nullptr,
             *type = nullptr;
  int num_tokens =
      settings_parse(&msg.setting[0], len, &section, &name, &value, &type);
  if (num_tokens < 0) {
    ROS_ERROR("Failed to parse settings %d", num_tokens);
    return;
  }
  // Store value.
  value_ = std::string(value);

  ROS_DEBUG("Read setting section: %s", section);
  ROS_DEBUG("Read setting name: %s", name);
  ROS_DEBUG("Read setting value: %s", value);
  ROS_DEBUG("Read setting type: %s", type);
}

void SettingsIo::receiveWriteResponse(const msg_settings_write_resp_t& msg,
                                      const uint8_t len) {
  write_success_ = msg.status == 0;

  // Parse settings.
  const char *section = nullptr, *name = nullptr, *value = nullptr,
             *type = nullptr;
  int num_tokens =
      settings_parse(&msg.setting[0], len - 1, &section, &name, &value, &type);
  if (num_tokens < 0) {
    ROS_ERROR("Failed to parse settings %d", num_tokens);
    return;
  }
  ROS_ERROR_COND(!write_success_, "Failed to write setting %s.%s.%s, %d",
                 section, name, value, msg.status);

  ROS_DEBUG("Write setting section: %s", section);
  ROS_DEBUG("Write setting name: %s", name);
  ROS_DEBUG("Write setting value: %s", value);
  ROS_DEBUG("Write setting type: %s", type);
}

bool SettingsIo::compareValue(const std::string& value) const {
  return value_.compare(value) == 0;
}
bool SettingsIo::checkBoolTrue() const { return value_.compare("True") == 0; }
std::string SettingsIo::getValue() const { return value_; }

bool SettingsIo::openConfig(const std::string& file) {
  ROS_INFO("Opening config: %s", file.c_str());
  return true;
}

}  // namespace piksi_multi_cpp
