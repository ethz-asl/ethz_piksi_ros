#!/bin/bash

#---------------- SBP ----------------
echo " "
echo "Installing SBP library for Piksi Multi."

GIT_REPO_LIBSBP=git@github.com:swift-nav/libsbp.git
REPO_TAG=v2.2.1 #version you want to checkout before installing

# Install libsbp in $HOME and compile it
mkdir -p ~/piksi_sbp_lib_multi
cd ~/piksi_sbp_lib_multi
git clone $GIT_REPO_LIBSBP
cd ./libsbp
git checkout $REPO_TAG

# Install requirements.
cd ./python
echo "Installing SBP dependencies."
sudo apt-get install pandoc
sudo pip install tox
sudo pip install -r requirements.txt
sudo pip install markupsafe
sudo python setup.py install
# Build package.
cd ..
sudo make python

# Remove temporary folder
echo "Removing temporary folder."
sudo rm -rf ~/piksi_sbp_lib_multi

echo "SBP Library Installed"

#---------------- Dialout Group ----------------
echo " "
echo "Do you wish to add the current user ($USER) to 'dialout' group? [y or Y to accept]"
read add_user_to_dialout

if [[ $add_user_to_dialout == "Y" || $add_user_to_dialout == "y" ]]; then

  if id -nG "$USER" | grep -qw dialout; then
    # user already belongs to dialout
    echo "User $USER was already included in 'dialout' group."
  else
    echo "Adding user $USER to 'dialout' group."
    sudo adduser $(whoami) dialout
    echo "Please logout and back in for the new group to take effect."
  fi
fi
