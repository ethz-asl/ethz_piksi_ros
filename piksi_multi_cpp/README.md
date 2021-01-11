# Installation
Install all [PPA dependencies](install/prepare-jenkins-slave.sh).
```
./ethz_piksi_ros/piksi_multi_cpp/install/prepare-jenkins-slave.sh
```

Create a **merged** catkin workspace.
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
catkin config --merge-devel
```

Download this package.
```
cd ~/catkin_ws/src
wstool init
wstool set --git ethz_piksi_ros git@github.com:ethz-asl/ethz_piksi_ros.git
wstool update
```

Next download all individual ROS package dependencies.
**Note**: If you have not setup [SSH keys in GitHub](https://help.github.com/en/enterprise/2.16/user/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) use [dependencies_https.rosinstall](install/dependencies_https.rosinstall).
```
wstool init
wstool merge ethz_piksi_ros/piksi_multi_cpp/install/dependencies.rosinstall
wstool update
```

Finally, build the workspace.
```
catkin build
```
