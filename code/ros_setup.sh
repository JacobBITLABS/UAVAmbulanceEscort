#This is not a difinitive installer, just the commands i found i needed in the correct order
#NEEDS: Ubuntu 20.04 or lower
#See http://wiki.ros.org/noetic/Installation/Ubuntu for a basic installation without PX4

#Write this to .bashrc
source /opt/ros/noetic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
#Write this to .bashrc
export ROS_WORKSPACE=~/catkin_ws/devel
source devel/setup.bash

cd ~
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

git clone https://github.com/PX4/PX4-Autopilot --recursive
#Write these to .bashrc
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins

sudo apt install python3-pip
sudo pip3 install kconfiglib
sudo pip3 install jinja2
sudo pip3 install jsonschema
sudo pip3 install future
#sudo pip3 install libxslt
#sudo apt install python-lxml python-libxml2
sudo apt install libgstreamer-plugins-base1.0-dev
cd ~/PX4-Autopilot
make
make px4_sitl gazebo

#TODO: Write project files to git also
#cp /mnt/UNI/DM884\ -\ Autonomous/UAVAmbulanceEscort/code/scripts/ros.py scripts/ros.py && roslaunch escort_py start_offb.launch