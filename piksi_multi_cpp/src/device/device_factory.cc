#include <piksi_multi_cpp/device/device.h>
#include <piksi_multi_cpp/device/device_dummy.h>
#include <piksi_multi_cpp/device/device_factory.h>
#include <piksi_multi_cpp/device/device_tcp.h>
#include <piksi_multi_cpp/device/device_usb.h>
#include <ros/console.h>
#include <regex>

namespace piksi_multi_cpp {

/* Creates device based on a unique identifier string
 * Examples:
 *  usb:// *                All USB devices (without space between slash + star)
 *  usb://<serial number>   USB device with given serial number
 *  tcp://<host>:<port>     TCP devices on that host with that port
 *
 *  To be implemented in the future:
 *  serial:///dev/ttyUSBx        Device on Serial port /dev/ttyUSBx
 *
 */
std::vector<Device::Ptr> DeviceFactory::createByIdentifier(
    const piksi_multi_cpp::Identifier& id) {
  std::regex rgx("^(.+):\\/\\/(.+)$");  // check if id is of type protocol://id

  std::smatch match;
  if (std::regex_search(id, match, rgx) && match.size() > 1) {
    // if string is match.
    std::string protocol = match.str(1);
    std::string protocol_address = match.str(2);

    if (protocol == "usb") {
      if (protocol_address == "*") {  // Autodiscovery of USB
        const Identifiers ids = DeviceUSB::discoverAllSerialNumbers();
        return createByIdentifiers(ids);  // recursive call of parent function.
      } else {                            // Specific USB device
        return {Device::Ptr(new DeviceUSB(protocol_address))};
      }
    } else if (protocol == "serial") {
      return {Device::Ptr(new DeviceSerial(protocol_address))};
    } else if (protocol == "tcp") {  // Specific TCP device
      return {Device::Ptr(new DeviceTCP(protocol_address))};
    } else if (protocol == "dummy") {
      return {Device::Ptr(new DeviceDummy(protocol_address))};
    } else {
      ROS_WARN_STREAM("Warning, unknown device Protocol " << protocol);
      return {};
    }
  } else {
    return {};
  }
}

std::vector<Device::Ptr> DeviceFactory::createByIdentifiers(
    const piksi_multi_cpp::Identifiers& ids) {
  std::vector<Device::Ptr> devices;
  for (const auto& id : ids) {
    ROS_INFO_STREAM(id);
    auto devs = createByIdentifier(id);
    // copy valid devices into devices vector
    std::copy_if(devs.begin(), devs.end(), std::back_inserter(devices),
                 [](auto dev_ptr) { return dev_ptr.get(); });
  }

  ROS_WARN_COND(devices.empty(), "No device created.");
  return devices;
}

std::vector<Device::Ptr> DeviceFactory::createAllDevicesByAutodiscovery() {
  return createByIdentifier("usb://*");
}

}  // namespace piksi_multi_cpp
