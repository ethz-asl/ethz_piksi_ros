#!/bin/bash
GPIO_PIN=250

DEVICE='/dev/ttyS5'
BAUD=230400

# Install PPS
cd ..
cd pps-gpio-modprobe
make clean
make
sudo cp pps-gpio-modprobe.ko /lib/modules/$(uname -r)/kernel/drivers/pps/clients/
sudo depmod
sudo sh -c 'echo "pps-gpio-modprobe" >> /etc/modules-load.d/10-pps-gpio-modprobe.conf'
sudo sh -c "echo 'options pps-gpio-modprobe gpio=${GPIO_PIN}' >> /etc/modprobe.d/10-pps-gpio-modprobe.conf"
cd ..

# Install NMEA GPS
sudo apt install gpsd -y
sudo cp cfg/gpsd /etc/default/gpsd
sudo dpkg-reconfigure gpsd

sudo sh -c "echo '#!/bin/bash' >> /etc/rc.local"
sudo sh -c "echo '' >> /etc/rc.local"
sudo sh -c "echo '# Setting GPS UART' >> /etc/rc.local"
sudo sh -c "echo 'stty -F ${DEVICE} ${BAUD}' >> /etc/rc.local"
sudo sh -c "echo '# Start GPSD' >> /etc/rc.local"
sudo sh -c "echo 'service gpsd start' >> /etc/rc.local"
sudo sh -c "echo '' >> /etc/rc.local"
sudo sh -c "echo 'exit 0' >> /etc/rc.local"

sudo chmod +x /etc/rc.local

# Install chrony.
sudo apt install chrony -y

sudo sh -c "echo '' >> /etc/chrony/chrony.conf"
sudo sh -c "echo '# GPS + PPS' >> /etc/chrony/chrony.conf"
sudo sh -c "echo 'refclock PPS /dev/pps0 lock NMEA' >> /etc/chrony/chrony.conf"
sudo sh -c "echo 'refclock SHM 0 offset 0.02 delay 0.2 refid NMEA' >> /etc/chrony/chrony.conf"

# Install PPS debug tools.
sudo apt install pps-tools gpsd-clients -y
