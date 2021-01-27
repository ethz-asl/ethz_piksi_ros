#!/bin/bash
if [ "$1" == "" ]; then
    echo "Please enter the PPS GPIO pin number, e.g., 250..."
    read GPIO_PIN
else
  GPIO_PIN="$1"
fi
echo "PPS is triggered on pin ${GPIO_PIN}."

if [ "$2" == "" ]; then
    echo "Please enter the NMEA UART device, e.g., ttyS5..."
    read DEVICE
else
  DEVICE="$2"
fi
echo "Using serial port /dev/${DEVICE}."

if [ "$3" == "" ]; then
    echo "Please enter the serial port baud rate, e.g., 115200..."
    read BAUD
else
  BAUD="$3"
fi
echo "Setting baud rate to ${BAUD}."

# Install PPS
cd ..
sudo apt install linux-headers-$(uname -r) libelf-dev
echo "Is your GPIO interruptable? [y or Y to accept]"
read is_interruptable
if [[ $is_interruptable == "Y" || $is_interruptable == "y" ]]; then
  KERNEL_MODULE=pps-gpio-modprobe
else
  echo "Please enter the PPS pulse width in milliseconds, e.g., 20..."
  read PULSE_WIDTH
  echo "Setting pulse width to ${PULSE_WIDTH}."
  KERNEL_MODULE=pps-gpio-poll
  PULSE_WIDTH_SETTING="pulse_width=${PULSE_WIDTH}"
fi
cd ${KERNEL_MODULE}
make clean
make
sudo cp *.ko /lib/modules/$(uname -r)/kernel/drivers/pps/clients/
sudo depmod
cd ..

# Install chrony.
sudo apt install chrony -y
sudo systemctl daemon-reload
sudo systemctl enable chrony.service
sudo systemctl disable systemd-timesyncd

echo "Do you wish to append a new PPS refclock to /etc/chrony/chrony.conf? [y or Y to accept]"
read append_chrony_conf
if [[ $append_chrony_conf == "Y" || $append_chrony_conf == "y" ]]; then
  sudo sh -c "tee -a /etc/chrony/chrony.conf << END

# GPS + PPS
refclock PPS /dev/pps0 lock NMEA
refclock SHM 0 offset 0.175 refid NMEA
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
After=pps.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/bin/sleep 20
ExecStartPre=/bin/stty -F /dev/${DEVICE} ${BAUD}
ExecStart=/usr/sbin/gpsd -n -r /dev/${DEVICE}

[Install]
WantedBy=multi-user.target
Also=gpsd.socket
END"
  sudo rm /etc/udev/rules.d/99-gpsd.rules
  sudo sh -c "tee -a /etc/udev/rules.d/99-gpsd.rules << END
SUBSYSTEM==\"tty\", KERNEL==\"${DEVICE}\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}+=\"gpsd.service\"
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable gpsd.service

# Install PPS
echo "Do you wish to configure pps? [y or Y to accept]"
read configure_pps
if [[ $configure_pps == "Y" || $configure_pps == "y" ]]; then
  echo "Configuring /etc/systemd/system/pps.service"
  sudo rm /etc/systemd/system/pps.service
  sudo sh -c "tee -a /etc/systemd/system/pps.service << END
[Unit]
Description=Modprobe pps gpio.
Before=chrony.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/modprobe ${KERNEL_MODULE} gpio=${GPIO_PIN} $PULSE_WIDTH_SETTING
ExecStop=/sbin/rmmod ${KERNEL_MODULE}

[Install]
WantedBy=multi-user.target
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable pps.service

# Install PPS debug tools.
sudo apt install pps-tools gpsd-clients -y

# Sign kernel module
# https://askubuntu.com/questions/762254/why-do-i-get-required-key-not-available-when-install-3rd-party-kernel-modules
echo "Do you wish to sign the kernel module? [y or Y to accept]"
read sign_kernel_module
if [[ $sign_kernel_module == "Y" || $sign_kernel_module == "y" ]]; then
  cd ~
  openssl req -new -x509 -newkey rsa:2048 -keyout MOK.priv -outform DER -out MOK.der -nodes -days 36500 -subj "/CN=Descriptive name/"
  sudo /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 ./MOK.priv ./MOK.der $(modinfo -n ${KERNEL_MODULE})
  sudo mokutil --import MOK.der
fi

echo "Please reboot to take changes into effect? [y or Y to accept]"
read reboot_now
if [[ $reboot_now == "Y" || $reboot_now == "y" ]]; then
  sudo reboot
fi
