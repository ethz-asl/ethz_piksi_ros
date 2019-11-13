# Install service.
echo "Do you wish to configure base station autostart? [y or Y to accept]"
read configure_base_autostart
if [[ $configure_base_autostart == "Y" || $configure_base_autostart == "y" ]]; then
  echo "Configuring /etc/systemd/system/base_station.service"
  sudo rm /etc/systemd/system/base_station.service
  sudo sh -c "tee -a /etc/systemd/system/base_station.service << END
[Unit]
Description=Start base station automatically on startup.

[Service]
Type=forking
ExecStart=/home/$USER/catkin_ws/src/ethz_piksi_ros/piksi_multi_cpp/install/startup_base_station.sh
Restart=on-failure
User=$USER

[Install]
WantedBy=multi-user.target
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable base_station
