# SWARMz

## Presentation

## Requirements
The swarmz_install.sh bash script file, installs on your linux computer the requirements for the current use of SwarmZ simulation and program.
To summurize, the following will be installed :
- ROS2
- Gazebo ignition transport 11
- ignition Gazebo 6
- Gazebo garden for ROS2 foxy and gazebo harmonic for ROS2 humble.
- ros-$ros-distro-actuator-msgs
- ros-$ros-distro-plotjuggler-ros
- libgflags-dev

Then it will build the following folders:
- ROS2 workspace of SwarmZ
- PX4-Autopilot
- Micro-XRCE-DDS-Agent

## Installation

## Usage

To run the simulation manually, so far, 13 terminals are needed.
1 for the Micro XRCE DDS agent
"MicroXRCEAgent udp4 -p 8888"
5 for the initialization of each of the 5 drones in gazebo
"cd PX4-Autopilot && export PX4_HOME_LAT=43.13471 && export PX4_HOME_LON=6.01507 && export PX4_HOME_ALT=6 && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='5,0' PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1" (for exemple for the 1st drone)
5 to run the offboard conotrol for each drone to make it fly
"ros2 run "
1 to run the acoustic simulation node to publish the sound topic of each drone in ROS2
1 to run the data plotter

cd SwarmZ_ROS2
source install/setup.bash
ros2 launch swarmz_control_py SwarmZ_simu.launch.py



To run the simulation manually, so far, 13 terminals are needed.
1 for the Micro XRCE DDS agent
5 for the initialization of each of the 5 drones in gazebo
5 to run the offboard conotrol for each drone to make it fly
1 to run the acoustic simulation node to publish the sound topic of each drone in ROS2
1 to run the data plotter