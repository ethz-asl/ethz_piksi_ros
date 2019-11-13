#ifndef PIKSI_MULTI_CPP_DEVICE_DEVICE_FACTORY_H_
#define PIKSI_MULTI_CPP_DEVICE_DEVICE_FACTORY_H_

#include "piksi_multi_cpp/device/device.h"

namespace piksi_multi_cpp {
/* A factory to help initiating devices at runtime. The factory gives interfaces
to the user on how he wants to create devices, e.g., autodiscovery, given a
serial number, interface or port.
See also http://www.blackwasp.co.uk/FactoryMethod.aspx for more details on
factories. */
class DeviceFactory {
 public:
  enum DeviceType { kUSB = 0, kTCP };

  // Method to create a device by Identifier String.
  static std::vector<Device::Ptr> createByIdentifier(const Identifier& id);

  // Method for multiple devices.
  static std::vector<Device::Ptr> createByIdentifiers(const Identifiers& id);


  // Factory method to create all devices autodiscovering all Piksis on all
  // interfaces.
  static std::vector<Device::Ptr> createAllDevicesByAutodiscovery();
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_DEVICE_FACTORY_H_
