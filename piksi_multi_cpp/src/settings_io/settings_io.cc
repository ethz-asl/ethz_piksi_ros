#include "piksi_multi_cpp/settings_io/settings_io.h"

#include <libsbp/settings.h>
#include <ros/console.h>
#include <cstring>
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
  if (!device_.get()) {
    ROS_ERROR("No device.");
    return false;
  }
  if (!device_->open()) {
    ROS_ERROR("Could not open device.");
    return false;
  }

  const uint16_t kSbpSenderId = 0x42;  // Device will only respond to this ID.

  // Send MSG_SETTINGS_READ_REQ
  const int kLen = section.size() + name.size() + 2;
  char setting[kLen];
  int cx = snprintf(&setting[0], kLen, "%s", section.c_str());
  if (cx < 0 || cx >= kLen) {
    ROS_ERROR("Error parsing setting %s.%s", section.c_str(), name.c_str());
    return false;
  }
  snprintf(&setting[cx + 1], kLen - cx, "%s", name.c_str());
  ROS_INFO_STREAM(setting);

  //
  //
  //  int cx = snprintf(setting, kLen, "%s", section.c_str());
  //  snprintf(setting + cx, kLen - cx, "%s", name.c_str());
  //  // strcpy(setting, section.c_str());
  //  // setting[section.size()] = '\0';
  //  // strncat(setting, name.c_str(), name.size() + 1);
  //
  //  // setting.copy(read_req.setting, setting.size() + 1);
  //  ROS_INFO("Requesting setting: %s", read_req.setting);
  //for (size_t i = 0; i < kLen;
  //     i++) { /* rotate upto \0 not 7 or random no of times */
  //  printf("%c", setting[i]);
  //}
  // for (size_t i = 0; i < section.size() + name.size() + 2; ++i) {
  //  std::cout << setting[i] << std::endl;
  //}
  msg_settings_read_req_t read_req;
  for (size_t i = 0; i < kLen; ++i) {
    read_req.setting[i] = setting[i];
    std::cout << setting[i] << std::endl;
  }

  std::cout << "kLen: " << kLen << std::endl;
  std::cout << "sizeof: " << sizeof(read_req) << std::endl;

  int req_success = sbp_send_message(
      state_.get(), SBP_MSG_SETTINGS_READ_REQ, kSbpSenderId, kLen,
      reinterpret_cast<uint8_t*>(&read_req), &SettingsIo::write_redirect);
  if (req_success != SBP_OK) {
    ROS_ERROR("Cannot request setting %s.%s, %d", section.c_str(), name.c_str(),
              req_success);
    return false;
  }

  auto start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < timeout_) {
    sbp_state_set_io_context(state_.get(), device_.get());
    int result =
        sbp_process(state_.get(), &piksi_multi_cpp::Device::read_redirect);
    if (result < 0) {
      ROS_WARN_STREAM("Error sbp_process: " << result);
    }
  }

  //// Wait for  MSG_SETTINGS_READ_RESP.
  //if (!settings_listener_.waitForCallback(timeout_)) {
  //  ROS_ERROR("Did not receive setting %s, %s.", section.c_str(), name.c_str());
  //  return false;
  //}

  device_->close();
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

  for (size_t i = 0; i < n; ++i) {
    ROS_INFO("%.2X", buff[i]);
  }

  // Execute instance's write function.
  int bytes_written =
      instance->device_->write(std::vector<uint8_t>(&buff[0], &buff[n]));
  std::cout << bytes_written << std::endl;
  return bytes_written;
}


int32_t SettingsIo::read_redirect(uint8_t* buff, uint32_t n, void* context) {
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
  // Execute instance's read function.
  return instance->device_->read(buff, n);
}

}  // namespace piksi_multi_cpp
