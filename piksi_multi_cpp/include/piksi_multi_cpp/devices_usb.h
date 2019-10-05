#ifndef PIKSI_MULTI_CPP_DEVICES_USB_H_
#define PIKSI_MULTI_CPP_DEVICES_USB_H_

#include <libusb-1.0/libusb.h>
#include <vector>
#include "piksi_multi_cpp/devices_base.h"

namespace piksi_multi_cpp {
class DevicesUSB : DevicesBase {
 public:
  DevicesUSB();

  bool open() override;
  bool read() override;
  bool close() override;

 private:
  bool identifyPiksi(libusb_device* dev);
  void printDeviceInfo(libusb_device_handle* dev);

  std::vector<libusb_device_handle*> handles_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICES_USB_H_
