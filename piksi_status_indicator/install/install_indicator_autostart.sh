# Install service.
echo "Do you wish to configure led status indicator autostart? [y or Y to accept]"
read configure_base_autostart
if [[ $configure_base_autostart == "Y" || $configure_base_autostart == "y" ]]; then
  echo "Configuring /etc/systemd/system/led_status_indicator.service"
  sudo rm /etc/systemd/system/led_status_indicator.service
  sudo sh -c "tee -a /etc/systemd/system/led_status_indicator.service << END
[Unit]
Description=Start led status indicator and rosserial connection with arduino board automatically on startup.

[Service]
Type=forking
ExecStart=/home/$USER/catkin_ws/src/ethz_piksi_ros/piksi_status_indicator/install/startup_led_indicator.sh
Restart=on-failure
User=$USER

[Install]
WantedBy=multi-user.target
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable led_status_indicator
