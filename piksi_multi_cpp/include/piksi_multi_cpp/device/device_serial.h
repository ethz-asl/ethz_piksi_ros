#ifndef PIKSI_MULTI_CPP_DEVICE_DEVICE_SERIAL_H_
#define PIKSI_MULTI_CPP_DEVICE_DEVICE_SERIAL_H_

#include <libserialport.h>
#include "piksi_multi_cpp/device/device.h"

namespace piksi_multi_cpp {

class DeviceSerial : public Device {
 public:
  /*
   * Identifier is expected to be of type "<address>@<baud>"
   */
  DeviceSerial(const Identifier& id);

  bool openImpl() override;
  int32_t readImpl(uint8_t* buff, uint32_t n) const override;
  int32_t writeImpl(std::vector<uint8_t> buff) const override;
  void closeImpl() override;

 protected:
  virtual bool allocatePort();
  virtual bool parseId();
  virtual bool setBaudRate();
  virtual bool setPortDefaults();
  struct sp_port* port_;

 private:
  std::string portname_;
  uint baudrate_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_DEVICE_SERIAL_H_
