#!/bin/bash
echo "Please enter the PPS GPIO pin number, e.g., 250?"
read GPIO_PIN
echo "PPS is triggered on pin ${GPIO_PIN}."

echo "Please enter the NMEA UART device, e.g., /dev/ttyS5?"
read DEVICE
echo "Using serial port ${DEVICE}."

echo "Please enter the serial port baud rate, e.g., 115200?"
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
sudo cp cfg/gpsd /etc/default/gpsd
echo "Do you wish to configure gpsd? [y or Y to accept]"
read configure_gpsd
if [[ $configure_gpsd == "Y" || $configure_gpsd == "y" ]]; then
  echo "Configuring /etc/default/gpsd"
  sudo rm /etc/default/gpsd
  sudo sh -c "echo '# Start the gpsd daemon automatically at boot time.' >> /etc/default/gpsd"
  sudo sh -c "echo 'START_DAEMON=\"true\"' >> /etc/default/gpsd"
  sudo sh -c "echo '# Use USB hotplugging to add new USB devices automatically to the daemon.' >> /etc/default/gpsd"
  sudo sh -c "echo 'USBAUTO=\"false\"' >> /etc/default/gpsd"
  sudo sh -c "echo '# Devices gpsd should collect to at boot time.' >> /etc/default/gpsd"
  sudo sh -c "echo '# They need to be read/writeable, either by user gpsd or the group dialout.' >> /etc/default/gpsd"
  sudo sh -c "echo 'DEVICES=\"${DEVICE}\"' >> /etc/default/gpsd"
  sudo sh -c "echo '# Other options you want to pass to gpsd.' >> /etc/default/gpsd"
  sudo sh -c "echo 'GPSD_OPTIONS=\"-n -r\"' >> /etc/default/gpsd"
  sudo sh -c "echo '' >> /etc/default/gpsd"
  sudo sh -c "echo 'GPSD_SOCKET=\"/var/run/gpsd.sock\"' >> /etc/default/gpsd"
  sudo chmod +x /etc/rc.local
fi

sudo dpkg-reconfigure gpsd

echo "Do you wish to create and overwrite a new /etc/rc.local? [y or Y to accept]"
read create_rc_local
if [[ $create_rc_local == "Y" || $create_rc_local == "y" ]]; then
  echo "Creating /etc/rc.local"
  sudo rm /etc/rc.local
  sudo sh -c "echo '#!/bin/bash' >> /etc/rc.local"
  sudo sh -c "echo '' >> /etc/rc.local"
  sudo sh -c "echo '# Start pps-gpio-modprobe module.' >> /etc/rc.local"
  sudo sh -c "echo 'modprobe pps-gpio-modprobe gpio=${GPIO_PIN}' >> /etc/rc.local"
  sudo sh -c "echo '# Setting GPS UART' >> /etc/rc.local"
  sudo sh -c "echo 'stty -F ${DEVICE} ${BAUD}' >> /etc/rc.local"
  sudo sh -c "echo '# Start GPSD' >> /etc/rc.local"
  sudo sh -c "echo 'service gpsd start' >> /etc/rc.local"
  sudo sh -c "echo '' >> /etc/rc.local"
  sudo sh -c "echo 'exit 0' >> /etc/rc.local"
  sudo chmod +x /etc/rc.local
fi

# Install chrony.
sudo apt install chrony -y

echo "Do you wish to append a new PPS refclock to /etc/chrony/chrony.conf? [y or Y to accept]"
read append_chrony_conf
if [[ $append_chrony_conf == "Y" || $append_chrony_conf == "y" ]]; then
  sudo sh -c "echo '' >> /etc/chrony/chrony.conf"
  sudo sh -c "echo '# GPS + PPS' >> /etc/chrony/chrony.conf"
  sudo sh -c "echo 'refclock PPS /dev/pps0 lock NMEA' >> /etc/chrony/chrony.conf"
  sudo sh -c "echo 'refclock SHM 0 delay 0.2 refid NMEA' >> /etc/chrony/chrony.conf"
fi

# Install PPS debug tools.
sudo apt install pps-tools gpsd-clients -y
