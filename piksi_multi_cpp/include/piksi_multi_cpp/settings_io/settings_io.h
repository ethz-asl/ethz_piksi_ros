#ifndef PIKSI_MULTI_CPP_SETTINGS_IO_SETTINGS_IO_H_
#define PIKSI_MULTI_CPP_SETTINGS_IO_SETTINGS_IO_H_

#include <libsbp/sbp.h>
#include <libsbp/settings.h>
#include <piksi_multi_cpp/device/device.h>
#include <memory>
#include <string>
#include <thread>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h"

namespace piksi_multi_cpp {

class SettingsIo {
 public:
  SettingsIo(const Device::Ptr& device);
  // Interface to read and write settings to device.
  bool readSetting(const std::string& section, const std::string& name);

  void printSetting(const msg_settings_read_by_index_resp_t& msg,
                    const uint8_t len);

  inline void setTimeout(const int timeout) { timeout_ = timeout; }

  // Simultaneous read and write with this class.
  static int32_t write_redirect(uint8_t* buff, uint32_t n, void* context);
  static int32_t read_redirect(uint8_t* buff, uint32_t n, void* context);

 private:
  Device::Ptr device_;
  std::shared_ptr<sbp_state_t> state_;
  int timeout_ = 1000;

  void process();
  std::thread process_thread_;
  std::atomic_bool thread_exit_requested_ = false;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SETTINGS_IO_SETTINGS_IO_H_
