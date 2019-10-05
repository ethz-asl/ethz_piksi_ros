#ifndef PIKSI_MULTI_CPP_DEVICES_USB_H_
#define PIKSI_MULTI_CPP_DEVICES_USB_H_

namespace piksi_multi_cpp {
class DevicesUSB {
 public:
  DevicesUSB();

  bool open() override;
  bool read() override;
  bool close() override;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICES_USB_H_
