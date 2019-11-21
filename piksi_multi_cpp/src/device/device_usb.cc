#include "piksi_multi_cpp/device/device_usb.h"

#include <libserialport.h>
#include <ros/console.h>

const int kInterfaceNumber = 1;

namespace piksi_multi_cpp {

DeviceUSB::DeviceUSB(const Identifier& id) : DeviceSerial(id) {}

bool DeviceUSB::parseId() {
  // not much to check with USB ids.
  return true;
}

bool DeviceUSB::setBaudRate() {
  // not necessary.
  return true;
}

Identifiers DeviceUSB::discoverAllSerialNumbers() {
  Identifiers identifiers;

  // Get list of all ports.
  struct sp_port** ports;
  sp_return result = sp_list_ports(&ports);

  if (result != SP_OK) {
    ROS_WARN_STREAM("No USB devices detected: " << result);
    return identifiers;
  }

  // Identify Piksi Multi among ports.
  for (int i = 0; ports[i]; i++) {
    // Check if port belongs to a Piksi Multi and get serial number.
    Identifier id = identifyPiksiAndGetSerialNumber(ports[i]);
    // Add serial to identifier set.
    if (!id.empty()) {
      identifiers.insert("usb://" + id);
    }
  }

  ROS_INFO("%lu Piksi Multi identified on USB.", identifiers.size());
  sp_free_port_list(ports);

  return identifiers;
}

void DeviceUSB::printPorts() {
  struct sp_port** ports;
  sp_return result = sp_list_ports(&ports);
  if (result == SP_OK) {
    for (int i = 0; ports[i]; i++) {
      printDeviceInfo(ports[i]);
    }
    sp_free_port_list(ports);
  } else {
    ROS_ERROR_STREAM("No serial devices detected: " << result);
  }
}

bool DeviceUSB::allocatePort() {
  // Find any serial port with serial number.
  struct sp_port** ports;
  sp_return result = sp_list_ports(&ports);
  if (result < 0) {
    ROS_ERROR_STREAM("No serial devices detected: " << result);
    return false;
  }

  for (int i = 0; ports[i]; i++) {
    auto sn_query = identifyPiksiAndGetSerialNumber(ports[i]);
    if (identifierEqual(sn_query, id_)) {
      result = sp_copy_port(ports[i], &port_);
      if (result == SP_OK) {
        break;
      } else {
        ROS_ERROR_STREAM("Cannot copy port: " << result);
      }
    }
  }

  sp_free_port_list(ports);

  if (!port_) {
    ROS_ERROR("Failed to allocate port.");
    return false;
  } else {
    ROS_INFO_STREAM(
        "Allocated USB port for Piksi Multi with serial number: " << id_);
    return true;
  }
}

Identifier DeviceUSB::identifyPiksiAndGetSerialNumber(struct sp_port* port) {
  if (!port) return Identifier();
  if (sp_get_port_transport(port) != sp_transport::SP_TRANSPORT_USB)
    return Identifier();

  // Get vendor and product id to identify Piksi Multi.
  int usb_vid = -1;
  int usb_pid = -1;
  sp_return result = sp_get_port_usb_vid_pid(port, &usb_vid, &usb_pid);
  if (result != SP_OK) {
    ROS_ERROR("Cannot get vendor ID and product ID for port %s: %d",
              sp_get_port_name(port), result);
    return Identifier();
  }
  std::tuple<uint16_t, uint16_t> usb_vidpid = {usb_vid, usb_pid};

  // These are the Piksi Multi USB identifiers. (see lsusb -v)
  std::tuple<uint16_t, uint16_t> kStandardPiksi = {0x2E69, 0x1001};
  std::tuple<uint16_t, uint16_t> kOldFirmware = {0x0525, 0xa4a7};

  // if this combination of vid/pid is in the set of valid tuples.
  if (usb_vidpid == kStandardPiksi) {
    // Get serial number as a unique identifier.
    char* serial_number = sp_get_port_usb_serial(port);
    return Identifier(serial_number);
  } else if (usb_vidpid == kOldFirmware) {
    ROS_ERROR_STREAM(
        "Piksi with old firmware detected (before 2.00). Update firmware in "
        "order to use this driver.");
    return Identifier();
    // return Identifier(devBoardId.str());
  } else {
    return Identifier();
  }
}

void DeviceUSB::printDeviceInfo(struct sp_port* port) {
  if (!port) return;

  ROS_INFO("Port: %s", sp_get_port_name(port));
  ROS_INFO("Manufacturer: %s", sp_get_port_usb_manufacturer(port));
  ROS_INFO("Product: %s", sp_get_port_usb_product(port));
  ROS_INFO("USB serial number: %s", sp_get_port_usb_serial(port));
  int usb_bus = -1;
  int usb_address = -1;
  sp_return result = sp_get_port_usb_bus_address(port, &usb_bus, &usb_address);
  if (result == SP_OK) {
    ROS_INFO("Bus: %d Adress: %d", usb_bus, usb_address);
  }
  int usb_vid = -1;
  int usb_pid = -1;
  result = sp_get_port_usb_vid_pid(port, &usb_vid, &usb_pid);
  if (result == SP_OK) {
    ROS_INFO("VendorID:ProductID: %4.0X:%4.0X", usb_vid, usb_pid);
  }
}

}  // namespace piksi_multi_cpp
