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

# Install NMEA GPS
sudo sh -c "echo '#!/bin/bash' >> /etc/rc.local"
sudo sh -c "echo '' >> /etc/rc.local"
sudo sh -c "echo '# Setting GPS UART' >> /etc/rc.local"
sudo sh -c "echo 'stty -F ${DEVICE} ${BAUD}' >> /etc/rc.local"
sudo sh -c "echo '' >> /etc/rc.local"
sudo sh -c "echo 'exit 0' >> /etc/rc.local"
