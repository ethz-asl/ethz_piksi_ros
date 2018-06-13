#!/bin/bash

GIT_REPO_LIBSBP=https://github.com/swift-nav/libsbp.git
REPO_TAG=v2.3.15 #version you want to checkout before installing

# Sort of reg express used to see if there is another verision on SBP library already installed
# Adapted from https://stackoverflow.com/questions/6363441/check-if-a-file-exists-with-wildcard-in-shell-script
SBP_PYTHON_DIRECTORY="/usr/local/lib/python2.7/dist-packages/sbp-*"

#---------------- SBP ----------------
echo " "
echo "Installing SBP library for Piksi Multi."

# Download libsbp in $HOME and compile it
STARTING_FOLDER=$(pwd)
mkdir -p ~/piksi_sbp_lib_multi
cd ~/piksi_sbp_lib_multi
git clone $GIT_REPO_LIBSBP
cd ./libsbp
git checkout $REPO_TAG

# Remove other Python libsbp, if any
if ls $SBP_PYTHON_DIRECTORY 1> /dev/null 2>&1; then
	echo " "
    echo "Another version of libsbp is already installed in the system."
    echo "Do you wish to remove it and install version $REPO_TAG ? [y or Y to accept]"
	read remove_other_libspb

	if [[ $remove_other_libspb == "Y" || $remove_other_libspb == "y" ]]; then
		echo "Removing previous libsbp."
		sudo rm -rf $SBP_PYTHON_DIRECTORY
	else
		echo "Installation procedure interrupted."
		exit 1
	fi

fi

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
cd $STARTING_FOLDER
echo "Removing temporary folder."
sudo rm -rf ~/piksi_sbp_lib_multi

echo "SBP Library Installed"

#---------------- Dialout Group ----------------
if id -nG "$USER" | grep -qw dialout; then
    echo "User $USER is already included in 'dialout' group."
else
    # $USER does not belong to dialout
    echo " "
	echo "Do you wish to add the current user ($USER) to 'dialout' group? [y or Y to accept]"
	read add_user_to_dialout

	if [[ $add_user_to_dialout == "Y" || $add_user_to_dialout == "y" ]]; then
	    echo "Adding user $USER to 'dialout' group."
	    sudo adduser $(whoami) dialout
	    echo "Please logout and back in for the new group to take effect."
	fi

fi

echo " "
echo "Installation completed."
