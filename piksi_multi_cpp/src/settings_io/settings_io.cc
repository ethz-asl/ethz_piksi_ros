#include "piksi_multi_cpp/settings_io/settings_io.h"

#include <libsbp/settings.h>
#include <libsettings/settings_util.h>
#include <ros/console.h>
#include <atomic>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

namespace piksi_multi_cpp {

SettingsIo::SettingsIo(const Device::Ptr& device)
    : device_(device), state_(std::make_shared<sbp_state_t>()) {
  sbp_state_init(state_.get());
  sbp_state_set_io_context(state_.get(), device_.get());
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

  // Parse request to format setting\0name\0
  size_t kLen = section.size() + name.size() + 2;
  msg_settings_read_req_t read_req;
  settings_format(section.c_str(), name.c_str(), nullptr, nullptr,
                  &read_req.setting[0], kLen);

  // Register setting listener.
  SBPLambdaCallbackHandler<msg_settings_read_by_index_resp_t> settings_listener(
      std::bind(&SettingsIo::printSetting, this, std::placeholders::_1,
                std::placeholders::_2),
      SBP_MSG_SETTINGS_READ_BY_INDEX_RESP, state_);

  // Start reading thread.
  thread_exit_requested_ = false;
  process_thread_ = std::thread(&SettingsIo::process, this);

  const uint16_t kSbpSenderId = 0x42;  // Device will only respond to this ID.
  msg_settings_read_by_index_req_t req;
  req.index = 0;
  std::cout << sizeof(req) << std::endl;
  int req_success = sbp_send_message(
      state_.get(), SBP_MSG_SETTINGS_READ_BY_INDEX_REQ, kSbpSenderId,
      sizeof(req), reinterpret_cast<uint8_t*>(&req), &Device::write_redirect);
  if (req_success != SBP_OK) {
    ROS_ERROR("Cannot request setting %s.%s, %d", section.c_str(), name.c_str(),
              req_success);
    device_->close();
    thread_exit_requested_.store(true);
    return false;
  }

  // Wait to receive setting.
  if (!settings_listener.waitForCallback(timeout_)) {
    ROS_ERROR("Did not receive setting %s, %s.", section.c_str(), name.c_str());
    thread_exit_requested_.store(true);
    if (process_thread_.joinable()) process_thread_.join();
    device_->close();
    return false;
  }

  thread_exit_requested_.store(true);
  if (process_thread_.joinable()) process_thread_.join();
  device_->close();
  return true;
}

void SettingsIo::printSetting(const msg_settings_read_by_index_resp_t& msg,
                              const uint8_t len) {
  uint8_t len_settings = len - 2;

  const char *section = nullptr, *name = nullptr, *value = nullptr,
             *type = nullptr;
  settings_parse(&msg.setting[0], len_settings, &section, &name, &value, &type);
  std::cout << section << std::endl;
  std::cout << name << std::endl;
  std::cout << value << std::endl;
  std::cout << type << std::endl;
}

void SettingsIo::process() {
  // Setting thread_exit_requested_ will terminate the thread.
  while (!thread_exit_requested_.load()) {
    if (!device_.get()) return;
    // Pass device read function to sbp_process.
    int result =
        sbp_process(state_.get(), &piksi_multi_cpp::Device::read_redirect);
    if (result < 0) {
      ROS_WARN_STREAM("Error sbp_process: " << result);
    }
  }
}

}  // namespace piksi_multi_cpp
