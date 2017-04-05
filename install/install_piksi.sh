#!/bin/bash

#---------------- SBP ----------------
echo " "
echo "Installing SBP library"

GIT_REPO_LIBSBP=git@github.com:swift-nav/libsbp.git
REPO_TAG=v1.2.1 #version you want to chechout before installing

# Install libsbp in $HOME and compile it
mkdir -p ~/software/piksi_sbp_lib
cd ~/software/piksi_sbp_lib
git clone $GIT_REPO_LIBSBP
cd ./libsbp
git checkout $REPO_TAG

# Install requirements.
cd ./python
echo "Installing SBP dependencies"
sudo apt-get install pandoc
sudo pip install tox
sudo pip install -r requirements.txt
sudo pip install markupsafe
sudo python setup.py install
# Build package.
cd ..
sudo make python

echo "SBP Library Installed"

# Export PYTHONPATH and make sure it points to the python subdirectory of the repository
sh -c 'echo "export PYTHONPATH=\${PYTHONPATH}:~/software/piksi_sbp_lib/libsbp/python #add libsbp for RTK GPS Piksi devices" >> ~/.bashrc'

#---------------- Udev Rule ----------------
echo " "
echo "Do you wish to create udev rule for piksi? [y or Y to accept]"
read create_udev_rule
if [[ $create_udev_rule == "Y" || $create_udev_rule == "y" ]]; then

  echo "Creating udev rule for Piksi"

  sudo touch /etc/udev/rules.d/99-piksi.rules
  sudo sh -c 'echo "#Piksi RTk GPS device" >> /etc/udev/rules.d/99-piksi.rules'
  sudo sh -c 'echo "SUBSYSTEMS==\"usb\", KERNEL==\"tty*\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", SYMLINK+=\"piksi\", MODE=\"0664\", GROUP=\"dialout\"" >> /etc/udev/rules.d/99-piksi.rules'
  sudo sh -c 'echo "SUBSYSTEMS==\"usb\", KERNEL==\"tty*\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"8398\", SYMLINK+=\"piksi\", MODE=\"0664\", GROUP=\"dialout\"" >> /etc/udev/rules.d/99-piksi.rules'
  sudo /etc/init.d/udev restart

  if id -nG "$USER" | grep -qw dialout; then
    # user already belongs to dialout
    echo "udev rule created correctly for Piksi"
  else
    echo "Adding user to 'dialout' group"
    sudo adduser $(whoami) dialout
    echo "Please logout and back in for the new group to take effect"
  fi
fi
