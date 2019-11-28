#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_FACTORY_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_FACTORY_H_

#include "piksi_multi_cpp/receiver/receiver.h"
#include "piksi_multi_cpp/receiver/settings_io.h"
#include "ros/ros.h"

namespace piksi_multi_cpp {
/* A factory to help initiating receivers at runtime. The factory gives
interfaces to the user on how he wants to create receivers, e.g., autodiscovery
of Piksi devices and autonaming of receivers or setting namespace, device and
receiver type manually.
See also http://www.blackwasp.co.uk/FactoryMethod.aspx for more details on
factories. */
class ReceiverFactory {
 public:
  /* The three types of receivers are

  kBaseStationReceiver: The static base station sending out RTK corrections.

  kPositionReceiver: The moving rover receiving RTK corrections from the
  base station and broadcasting RTK GPS positions.

  kAttitudeReceiver: The moving rover receiving RTK corrections from a moving
  reference receiver and broadcasting the moving baseline (also referred to as
  heading). */
  enum ReceiverType {
    kBaseStationReceiver = 0,
    kPositionReceiver,
    kAttitudeReceiver,
    kSettingIo,  // An abstract receiver that allows reading and writing
                 // settings.
    kUnknown,
    kError // Occurs when unable to open device.
  };
  // Factory method to create a receiver by setting node handle, hardware
  // device, and receiver type.
  // Warning: Node handle namespace must be unique for every receiver.
  static Receiver::Ptr createReceiverByReceiverType(const ros::NodeHandle& nh,
                                                    const Device::Ptr& device,
                                                    const ReceiverType type);

  // Factory method to create a receiver by setting node handle and hardware
  // device. Receiver type is inferred automatically.
  // Warning: Node handle namespace must be unique for every receiver.
  static Receiver::Ptr createReceiverByDevice(const ros::NodeHandle& nh,
                                              const Device::Ptr& device);

  // Create all receivers from node handle only. Autodetects connected hardware
  // devices, infers device type from Piksi firmware settings and assigns unique
  // name spaces.
  static std::vector<Receiver::Ptr> createAllReceiversByIdentifiersAndNaming(
      const ros::NodeHandle& nh, const Identifiers& ids);

  // A convinience function to open all devices in ids as SettingIo receivers to
  // write the settings.
  static std::vector<SettingsIo::Ptr> createSettingIoReceivers(
      const Identifiers& ids);

 private:
  // Infer receiver type from Piksi firmware settings.
  static ReceiverType inferType(const Device::Device::Ptr& dev);
  static std::string createNameSpace(const ReceiverType type, const size_t id);
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_FACTORY_H_
