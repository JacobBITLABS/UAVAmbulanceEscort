# UAV Ambulance Escort 
This is a conceptual work for a exam project

# Installation

## Pixhark - PIX4 and Gazebo Simulation
Clone PixHawk Flight Computer repository
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
Install using following command:
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
Restart your system, on linux:
```
sudo reboot now
```
## QTGroundControl
This is essentially not necesarry, but is an easy monitoring tool. Download image from following 



# Running 
To run the project use the following commands

## QGroundControl 
./QGroundControl.AppImage

## Start PX4 and Gazebo:
```
make px4_sitl gazebo
```