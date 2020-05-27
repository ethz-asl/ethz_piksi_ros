#!/bin/bash

echo "Setup automatic interface startup."

echo "Please enter the PPS gpiochip, e.g., gpiochip3..."
read GPIOCHIP

echo "Do you wish to configure UDEV rule for the gpiochip and add user to group gpio? [y or Y to accept]"
read create_udev
if [[ $create_udev == "Y" || $create_udev == "y" ]]; then
  echo "Please enter the PPS gpiochip, e.g., gpiochip3..."
  read GPIOCHIP

  sudo rm /etc/udev/rules.d/99-gpio.rules
  sudo sh -c "tee -a /etc/udev/rules.d/99-gpio.rules << END
KERNEL==\"${GPIOCHIP}\", GROUP=\"gpio\"
END"

  sudo addgroup gpio
  sudo usermod -a -G gpio ${USER}
fi

# Autostart interface
echo "Do you wish to configure interface autostart? [y or Y to accept]"
read configure_autostart
if [[ $configure_autostart == "Y" || $configure_autostart == "y" ]]; then
  echo "Configuring /etc/systemd/system/piksi_interface.service"
  sudo rm /etc/systemd/system/piksi_interface.service
  sudo sh -c "tee -a /etc/systemd/system/piksi_interface.service << END
[Unit]
Description=Piksi interface (push button, status LED)
After=piksi.service

[Service]
Type=forking
ExecStart=/home/$USER/catkin_ws/src/ethz_piksi_ros/piksi_multi_interface/install/startup_interface.sh
User=$USER

[Install]
WantedBy=multi-user.target
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable piksi_interface.service

echo "Please reboot to take changes into effect? [y or Y to accept]"
read reboot_now
if [[ $reboot_now == "Y" || $reboot_now == "y" ]]; then
  sudo reboot
fi
