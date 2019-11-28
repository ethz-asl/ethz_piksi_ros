#ifndef PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_
#define PIKSI_MULTI_CPP_RECEIVER_SETTINGS_IO_H_

#include "piksi_multi_cpp/receiver/receiver.h"

#include <libsbp/settings.h>
#include <memory>
#include <string>

namespace piksi_multi_cpp {

enum SettingWriteStatusValues {
  kAccepted = 0,
  kRecejectedValueUnparsable = 1,
  kRecejectedDoesNotExist = 2,
  kRecejectedNameUnparsable = 3,
  kRecejectedReadOnly = 4,
  kRecejectedModificationDisabled = 5,
  kRecejectedUnspecified = 6
};

class SettingsIo : public Receiver {
 public:
  typedef std::shared_ptr<SettingsIo> Ptr;
  SettingsIo(const Device::Ptr& device);

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
  // Open, parse and write a piksi config .ini file to a device.
  bool updateConfig(const std::string& file);

  // https://github.com/swift-nav/libsettings/blob/master/include/libsettings/settings_util.h#L20-L30
  enum settings_tokens_e {
    SETTINGS_TOKENS_INVALID = -1, /** An error occurred */
    SETTINGS_TOKENS_EMPTY = 0,    /** No tokens found */
    SETTINGS_TOKENS_SECTION = 1,  /** Section token found */
    SETTINGS_TOKENS_NAME = 2,     /** Section and name tokens found */
    SETTINGS_TOKENS_VALUE = 3,    /** Section, name and value tokens found */
    SETTINGS_TOKENS_TYPE = 4, /** Section, name, value and type tokens found */
    SETTINGS_TOKENS_EXTRA_NULL =
        5, /** Section, name, value and type tokens found,
               this is for backwards compatibility for FW
               versions 2.2 and older */
  };

  // Parse settings message from section, name, value, type.
  // https://github.com/swift-nav/libsettings/blob/master/include/libsettings/settings_util.h#L36-L41
  static int formatSettings(const char* section, const char* name,
                            const char* value, const char* type, char* buf,
                            size_t blen);

  // Parse settings section, name, value, type from message.
  // https://github.com/swift-nav/libsettings/blob/master/include/libsettings/settings_util.h#L61-L66
  static settings_tokens_e parseSettings(const char* buf, size_t blen,
                                         const char** section,
                                         const char** name, const char** value,
                                         const char** type);

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
