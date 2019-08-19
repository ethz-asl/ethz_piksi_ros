#!/bin/bash
echo "Please enter the PPS GPIO pin number, e.g., 250..."
read GPIO_PIN
echo "PPS is triggered on pin ${GPIO_PIN}."

echo "Please enter the NMEA UART device, e.g., /dev/ttyS5..."
read DEVICE
echo "Using serial port ${DEVICE}."

echo "Please enter the serial port baud rate, e.g., 115200..."
read BAUD
echo "Setting baud rate to ${BAUD}."

# Install PPS
cd ..
cd pps-gpio-modprobe
sudo apt-get install linux-headers-$(uname -r)
make clean
make
sudo cp pps-gpio-modprobe.ko /lib/modules/$(uname -r)/kernel/drivers/pps/clients/
sudo depmod
cd ..

# Install NMEA GPS
sudo apt install gpsd -y
echo "Do you wish to configure gpsd? [y or Y to accept]"
read configure_gpsd
if [[ $configure_gpsd == "Y" || $configure_gpsd == "y" ]]; then
  echo "Configuring /etc/default/gpsd"
  sudo rm /etc/default/gpsd
  sudo sh -c "tee -a /etc/default/gpsd << END
# Start the gpsd daemon automatically at boot time.
START_DAEMON=\"true\"
# Use USB hotplugging to add new USB devices automatically to the daemon.
USBAUTO=\"false\"
# Devices gpsd should collect to at boot time.
# They need to be read/writeable, either by user gpsd or the group dialout.
DEVICES=\"${DEVICE}\"
# Other options you want to pass to gpsd.
GPSD_OPTIONS=\"-n -r\"
GPSD_SOCKET=\"/var/run/gpsd.sock\"
END"
fi

sudo dpkg-reconfigure gpsd

echo "Do you wish to create and overwrite a new /etc/rc.local? [y or Y to accept]"
read create_rc_local
if [[ $create_rc_local == "Y" || $create_rc_local == "y" ]]; then
  echo "Creating /etc/rc.local"
  sudo rm /etc/rc.local
  sudo sh -c "tee -a /etc/rc.local << END
#!/bin/bash

# Start pps-gpio-modprobe module.
modprobe pps-gpio-modprobe gpio=${GPIO_PIN}
# Setting GPS UART
service gpsd stop
stty -F ${DEVICE} ${BAUD}
service gpsd start

exit 0
END"
  sudo chmod +x /etc/rc.local
fi

# Install chrony.
sudo apt install chrony -y

echo "Do you wish to append a new PPS refclock to /etc/chrony/chrony.conf? [y or Y to accept]"
read append_chrony_conf
if [[ $append_chrony_conf == "Y" || $append_chrony_conf == "y" ]]; then
  sudo sh -c "tee -a /etc/chrony/chrony.conf << END

# GPS + PPS
refclock PPS /dev/pps0 lock NMEA
refclock SHM 0 delay 0.2 refid NMEA
END"
fi

# Install PPS debug tools.
sudo apt install pps-tools gpsd-clients -y
