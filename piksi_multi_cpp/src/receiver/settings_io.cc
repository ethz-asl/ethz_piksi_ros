#include "piksi_multi_cpp/receiver/settings_io.h"

#include <libsbp/settings.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <chrono>
#include <fstream>
#include <regex>
#include <thread>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h"

const int kSleepMs = 30;

namespace piksi_multi_cpp {

SettingsIo::SettingsIo(const Device::Ptr& device) : Receiver(device) {}

bool SettingsIo::readSetting(const std::string& section,
                             const std::string& name, const int timeout_ms) {
  value_.clear();

  // Parse request to format setting\0name\0
  size_t kLen = section.size() + name.size() + 2;
  char read_req[kLen] = {0};
  formatSettings(section.c_str(), name.c_str(), nullptr, nullptr, read_req,
                 kLen);

  // Register setting listener.
  SBPLambdaCallbackHandler<msg_settings_read_resp_t> settings_listener(
      std::bind(&SettingsIo::receiveReadResponse, this, std::placeholders::_1,
                std::placeholders::_2),
      SBP_MSG_SETTINGS_READ_RESP, state_);

  // Send message.
  // The reading sometimes fails with unspecified error. We wait a little.
  std::this_thread::sleep_for(std::chrono::milliseconds(kSleepMs));
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
  size_t kLen = section.size() + name.size() + value.size() + 3;
  char write_req[kLen] = {0};
  formatSettings(section.c_str(), name.c_str(), value.c_str(), nullptr,
                 write_req, kLen);

  // Register setting listener.
  SBPLambdaCallbackHandler<msg_settings_write_resp_t> write_resp_listener(
      std::bind(&SettingsIo::receiveWriteResponse, this, std::placeholders::_1,
                std::placeholders::_2),
      SBP_MSG_SETTINGS_WRITE_RESP, state_);

  // Send message.
  // The writing sometimes fails with unspecified error. We wait a little.
  std::this_thread::sleep_for(std::chrono::milliseconds(kSleepMs));
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

  // Save to persistent memory.
  char empty_req[0];
  int save_success =
      sbp_send_message(state_.get(), SBP_MSG_SETTINGS_SAVE, SBP_SENDER_ID, 0,
                       (unsigned char*)(&empty_req), &Device::write_redirect);
  if (save_success != SBP_OK) {
    ROS_ERROR("Cannot save setting %s.%s.%s, %d", section.c_str(), name.c_str(),
              value.c_str(), req_success);
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
      parseSettings(&msg.setting[0], len, &section, &name, &value, &type);
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
      parseSettings(&msg.setting[0], len - 1, &section, &name, &value, &type);
  if (num_tokens < 0) {
    ROS_ERROR("Failed to parse settings %d", num_tokens);
    return;
  }

  std::string setting;
  if (section) setting += std::string(section);
  if (name) setting += "." + std::string(name);
  if (value) setting += "." + std::string(value);
  if (type) setting += "." + std::string(type);

  switch (msg.status) {
    case SettingWriteStatusValues::kAccepted:
      ROS_DEBUG("Accepted; value updated %s", setting.c_str());
      break;
    case SettingWriteStatusValues::kRecejectedValueUnparsable:
      ROS_ERROR("Rejected; value unparsable or out-of-range %s",
                setting.c_str());
      break;
    case SettingWriteStatusValues::kRecejectedDoesNotExist:
      ROS_ERROR("Rejected; requested setting does not exist %s",
                setting.c_str());
      break;
    case SettingWriteStatusValues::kRecejectedNameUnparsable:
      ROS_ERROR("Rejected; setting name could not be parsed %s",
                setting.c_str());
      break;
    case SettingWriteStatusValues::kRecejectedReadOnly:
      ROS_DEBUG("Rejected; setting is read only %s", setting.c_str());
      break;
    case SettingWriteStatusValues::kRecejectedModificationDisabled:
      ROS_ERROR("Rejected; modification is temporarily disabled %s",
                setting.c_str());
      break;
    case SettingWriteStatusValues::kRecejectedUnspecified:
      ROS_ERROR("Rejected; unspecified error %s", setting.c_str());
      break;
    default:
      ROS_ERROR("Rejected; unknown error %d, %s", msg.status, setting.c_str());
  }
}

bool SettingsIo::compareValue(const std::string& value) const {
  return value_ == value;
}
bool SettingsIo::checkBoolTrue() const { return compareValue("True"); }
std::string SettingsIo::getValue() const { return value_; }

bool SettingsIo::updateConfig(const std::string& file) {
  ROS_INFO("Opening config: %s", file.c_str());

  // Open.
  std::ifstream in_file;
  in_file.open(file);
  if (!in_file) {
    ROS_ERROR("Cannot open file: %s", file.c_str());
    return false;
  }

  // Compare firmware version.
  std::regex rgx_firmware("firmware_version\\ \\=\\ (.+)");
  std::smatch firmware_matches;
  std::string firmware_config;
  std::string line;
  while (std::getline(in_file, line)) {
    if (std::regex_search(line, firmware_matches, rgx_firmware)) {
      if (firmware_matches.size() > 1) firmware_config = firmware_matches[1];
      ROS_DEBUG("%s", firmware_config.c_str());
      break;
    }
  }
  in_file.seekg(0);  // Reset file pointer.
  if (!readSetting("system_info", "firmware_version")) {
    ROS_ERROR("Cannot read firmware version setting.");
    return false;
  }

  if (!compareValue(firmware_config)) {
    ROS_ERROR(
        "Firmware version in config file %s does not match firmware version on "
        "device %s. Please update this driver and use Swift console to update "
        "device firmware.",
        firmware_config.c_str(), value_.c_str());
    return false;
  }

  // Parse and write setting.
  std::string section;
  while (std::getline(in_file, line)) {
    // Find section.
    std::regex rgx_section("\\[(.*)\\]");  // Group between square brackets.
    std::smatch section_matches;
    if (std::regex_search(line, section_matches, rgx_section)) {
      if (section_matches.size() > 1) section = section_matches[1];
      continue;  // New section.
    }
    if (section.empty()) continue;
    // Ignore read only settings.
    if (section == "system_info") continue;
    // TODO(rikba): Find a way to modify the connections if necessary.
    if (section == "ethernet") continue;
    if (section == "usb0") continue;

    // Find name.
    std::regex rgx_name(".+?(?=\\ \\=)");  // Up to whitespace equal.
    std::smatch name_matches;
    std::string name;
    if (std::regex_search(line, name_matches, rgx_name)) {
      name = name_matches[0];
    } else {
      continue;  // No name found.
    }

    // Find value.
    std::regex rgx_value(
        "\\ \\=\\ (.+)");  // After whitespace equal whitespace combination.
    std::smatch value_matches;
    std::string value;
    if (std::regex_search(line, value_matches, rgx_value)) {
      if (value_matches.size() > 1) value = value_matches[1];
    }

    // Write setting.
    ROS_DEBUG("Writing setting %s.%s.%s", section.c_str(), name.c_str(),
              value.c_str());
    writeSetting(section, name, value);
  }
  return true;
}

int SettingsIo::formatSettings(const char* section, const char* name,
                               const char* value, const char* type, char* buf,
                               size_t blen) {
  int n = 0;
  int l = 0;

  const char* tokens[] = {section, name, value, type};

  for (uint8_t i = 0; i < sizeof(tokens) / sizeof(tokens[0]); ++i) {
    const char* token = tokens[i];

    if (token == NULL) {
      break;
    }

    l = snprintf(&buf[n], blen - n, "%s", token);

    if ((l < 0) || ((size_t)l >= blen - n)) {
      return -1;
    }

    n += l + 1;
  }

  return n;
}

SettingsIo::settings_tokens_e SettingsIo::parseSettings(
    const char* buf, size_t blen, const char** section, const char** name,
    const char** value, const char** type) {
  if (section) *section = NULL;
  if (name) *name = NULL;
  if (value) *value = NULL;
  if (type) *type = NULL;

  /* All strings must be NULL terminated */
  if ((blen > 0) && (buf[blen - 1] != '\0')) {
    return SETTINGS_TOKENS_INVALID;
  }

  const char** tokens[] = {section, name, value, type};
  settings_tokens_e tok = SETTINGS_TOKENS_EMPTY;
  size_t str_start = 0;
  for (size_t idx = 0; idx < blen; ++idx) {
    if (buf[idx] != '\0') {
      continue;
    }
    if ((size_t)tok < sizeof(tokens) / sizeof(tokens[0]) &&
        tokens[tok] != NULL) {
      *(tokens[tok]) = &buf[str_start];
    }
    str_start = idx + 1;
    tok = static_cast<settings_tokens_e>(static_cast<int>(tok) + 1);
  }

  return (tok <= SETTINGS_TOKENS_EXTRA_NULL) ? tok : SETTINGS_TOKENS_INVALID;
}

}  // namespace piksi_multi_cpp
