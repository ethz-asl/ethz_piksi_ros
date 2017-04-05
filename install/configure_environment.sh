#!/bin/bash
GIT_REPO_LIBSBP=git@github.com:swift-nav/libsbp.git
REPO_TAG=v1.2.1 #version you want to chechout before installing

# Install libsbp in $HOME and compile it
mkdir  ~/piksi_rtk_lib
cd ~/piksi_rtk_lib
git clone $GIT_REPO_LIBSBP
cd ./libsbp
git checkout $REPO_TAG

# Install requirements.
cd ./python
sudo apt-get install pandoc
sudo pip install tox
sudo pip install -r requirements.txt
sudo pip install markupsafe
sudo python setup.py install
# Build package.
cd ..
sudo make python

echo "Build completed"

# Export PYTHONPATH and make sure it points to the python subdirectory of the repository
sh -c 'echo "export PYTHONPATH=\${PYTHONPATH}:~/piksi_rtk_lib/libsbp/python #add libsbp for RTK GPS Piksi devices" >> ~/.bashrc'
