#ifndef PIKSI_MULTI_CPP_DEVICE_USB_H_
#define PIKSI_MULTI_CPP_DEVICE_USB_H_

#include <libserialport.h>
#include <cstring>
#include <set>
#include <vector>
#include "piksi_multi_cpp/device.h"

namespace piksi_multi_cpp {

typedef char* USBIdentifier;
struct identifierless {
  bool operator()(USBIdentifier a, USBIdentifier b) { return strcmp(a, b) < 0; }
};
typedef std::set<USBIdentifier, identifierless> USBIdentifiers;

class DeviceUSB : public Device {
 public:
  DeviceUSB(const USBIdentifier& id);

  // List all unique Piksi multi devices using the usb serial number.
  static USBIdentifiers getAllIdentifiers();
  bool open() override;
  int32_t read(uint8_t* buff, uint32_t n, void* context) override;
  void close() override;

 private:
  static USBIdentifier identifyPiksi(struct sp_port* port);
  static void printDeviceInfo(struct sp_port* port);
  static void printPorts();
  bool allocatePort(const USBIdentifier& id);

  struct sp_port* port_;
  USBIdentifier id_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_USB_H_
