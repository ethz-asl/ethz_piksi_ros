# Install service.
echo "Do you wish to configure led status indicator autostart? [y or Y to accept]"
read configure_base_autostart
if [[ $configure_base_autostart == "Y" || $configure_base_autostart == "y" ]]; then
  SERVICE_FILE=/etc/systemd/system/led_status_indicator.service
  if [[ -f "$SERVICE_FILE" ]]; then
    echo "Reconfiguring /etc/systemd/system/led_status_indicator.service"
    sudo rm /etc/systemd/system/led_status_indicator.service
  else
    echo "Creating new service in /etc/systemd/system/led_status_indicator.service"
  fi

  sudo sh -c "tee -a /etc/systemd/system/led_status_indicator.service << END

[Unit]
Description=Start led status indicator and rosserial connection with arduino board automatically on startup.
After=base_station.service

[Service]
Type=forking
ExecStart=/home/$USER/catkin_ws/src/ethz_piksi_ros/piksi_status_indicator/install/startup_led_indicator.sh
Restart=on-failure
User=$USER

[Install]
WantedBy=multi-user.target
END"

sudo systemctl daemon-reload
sudo systemctl enable led_status_indicator
fi

echo "Would you like to set up an udev rule for your arduino? [y or Y to accept]"
read configure_udev_rules
if [[ $configure_udev_rules == "Y" || $configure_udev_rules == "y" ]]; then
  echo "Creating new udev rule. The arduino will then be assigned the device path: /dev/status_leds"
  sudo cp /home/$USER/catkin_ws/src/ethz_piksi_ros/piksi_status_indicator/install/98-status_leds.rules /etc/udev/rules.d/.
fi
