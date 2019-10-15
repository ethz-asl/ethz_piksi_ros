#ifndef PIKSI_MULTI_CPP_DEVICE_DEVICE_USB_H_
#define PIKSI_MULTI_CPP_DEVICE_DEVICE_USB_H_

#include <libserialport.h>
#include "device_serial.h"
#include "piksi_multi_cpp/device/device.h"

namespace piksi_multi_cpp {

class DeviceUSB : public DeviceSerial {
 public:
  DeviceUSB(const Identifier& id);

  // List all unique Piksi multi devices using the usb serial number.
  static Identifiers discoverAllSerialNumbers();

 private:
  static Identifier identifyPiksiAndGetSerialNumber(struct sp_port* port);
  static void printDeviceInfo(struct sp_port* port);
  static void printPorts();
  bool allocatePort() override ;
  bool parseId() override ;
  bool setBaudRate() override;

};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_DEVICE_USB_H_
