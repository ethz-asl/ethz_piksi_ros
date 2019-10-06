#ifndef PIKSI_MULTI_CPP_DEVICE_USB_H_
#define PIKSI_MULTI_CPP_DEVICE_USB_H_

#include <libusb-1.0/libusb.h>
#include <vector>
#include "piksi_multi_cpp/device_base.h"

namespace piksi_multi_cpp {
class DeviceUSB : DeviceBase {
 public:
  DeviceUSB();

  bool open() override;
  bool read() override;
  bool close() override;

 private:
  bool identifyPiksi(libusb_device* dev);
  void printDeviceInfo(libusb_device_handle* dev);

  libusb_device_handle* handle_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_USB_H_
