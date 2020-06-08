# Install service.
echo "Do you wish to configure piksi autostart? [y or Y to accept]"
read configure_base_autostart
if [[ $configure_base_autostart == "Y" || $configure_base_autostart == "y" ]]; then
  echo "Is this a base station? [y or Y]"
  read is_base
  TYPE=rover
  if [[ $is_base == "Y" || $is_base == "y" ]]; then
    TYPE=base_station
  fi

  echo "Configuring /etc/systemd/system/piksi.service"
  sudo rm /etc/systemd/system/piksi.service
  sudo sh -c "tee -a /etc/systemd/system/piksi.service << END
[Unit]
Description=Start piksi automatically on startup.
After=network-online.target

[Service]
Type=forking
ExecStartPre=/bin/sleep 120
ExecStart=/home/$USER/catkin_ws/src/ethz_piksi_ros/piksi_multi_cpp/install/startup_${TYPE}.sh
Restart=on-failure
User=$USER
TimeoutStartSec=121

[Install]
WantedBy=multi-user.target
END"
fi

sudo systemctl daemon-reload
sudo systemctl enable piksi
