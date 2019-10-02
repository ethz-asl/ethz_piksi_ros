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
sudo apt install linux-headers-$(uname -r) libelf-dev
make clean
make
sudo cp pps-gpio-modprobe.ko /lib/modules/$(uname -r)/kernel/drivers/pps/clients/
sudo depmod
cd ..

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

# Install NMEA GPS
sudo apt install gpsd -y
echo "Do you wish to configure gpsd? [y or Y to accept]"
read configure_gpsd
if [[ $configure_gpsd == "Y" || $configure_gpsd == "y" ]]; then
  echo "Configuring /etc/systemd/system/gpsd.service"
  sudo rm /etc/systemd/system/gpsd.service
  sudo sh -c "tee -a /etc/systemd/system/gpsd.service << END
[Unit]
Description=GPS (Global Positioning System) Daemon
Requires=gpsd.socket

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/bin/stty -F ${DEVICE} ${BAUD}
ExecStart=/usr/sbin/gpsd -n -r ${DEVICE}

[Install]
WantedBy=multi-user.target
WantedBy=chrony.service
Also=gpsd.socket
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable gpsd.service

# Install PPS
echo "Do you wish to configure pps? [y or Y to accept]"
read configure_pps
if [[ $configure_pps == "Y" || $configure_pps == "y" ]]; then
  echo "Configuring /etc/systemd/system/pps-modprobe.service"
  sudo rm /etc/systemd/system/pps-modprobe.service
  sudo sh -c "tee -a /etc/systemd/system/pps-modprobe.service << END
[Unit]
Description=Modprobe pps gpio.

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/modprobe pps-gpio-modprobe gpio=${GPIO_PIN}
ExecStop=/sbin/rmmod pps-gpio-modprobe

[Install]
WantedBy=multi-user.target
WantedBy=chrony.service
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable pps-gpio-modprobe.service

# Install PPS debug tools.
sudo apt install pps-tools gpsd-clients -y
