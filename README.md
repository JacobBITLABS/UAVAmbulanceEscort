# UAV Ambulance Escort 
This is a conceptual work for a exam project

# Installation

## Pixhark - PIX4 and Gazebo Simulation
Clone PixHawk Flight Computer repository
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
Install using following commands:
```
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
cd ~/PX4-Autopilot
make
```
Restart your system, on linux:
```
sudo reboot now
```
## QTGroundControl
This is essentially not necesarry, but is an easy monitoring tool. Download image from following: 
http://qgroundcontrol.com/downloads/

# Running 
To run the project use the following commands

## QGroundControl 
./QGroundControl.AppImage

## Start PX4 and Gazebo:
```
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
export PX4_HOME_LAT=55.35869
export PX4_HOME_LON=10.34174
export PX4_HOME_ALT=489

cd ~/PX4-Autopilot/Tools/simulation/gazebo
./sitl_multiple_run.sh -m iris -n 2
```

## Start the program
```
git clone https://github.com/JacobBITLABS/UAVAmbulanceEscort
cd UAVAmbulanceEscort\code
git clone https://github.com/JacobBITLABS/MAVFleetControl
```
Replace the api keys in droneDirection2.py with your own
```
pip3 install mavsdk geopy
python3 GeneralManager.py
```