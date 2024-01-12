# SWARMz

## Presentation

The aim of this challenge is to propose a framework for showcasing the use of a swarm of drones to secure a perimeter. The scenario chosen for this challenge is that of securing a stadium during a sporting event. The drones will have to position themselves above the stadium and detect blasts using their microphone, then locate the sources of these blasts by triangulation between the different drones.

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

The following steps will install and build packages mentioned above

```bash
git clone https://github.com/nbaudesson/SWARMz.git --recursive
cd SWARMz/SwarmZ_ROS2
source swarmz_install.sh
```

## Usage

***To simplify***

**1.** Run the simulation with:
```bash
source install/setup.bash
ros2 launch swarmz_control_py SwarmZ_simu.launch.py
```
*Alternativly, to run the simulation without the gazebo GUI:*
```bash
source install/setup.bash
ros2 launch swarmz_control_py SwarmZ_simu.launch.py headless:=1
```

**2.** Run a scenario for the simulated noises with:
```bash
source install/setup.bash
ros2 launch acoustics_simulation_py acoustics_bringup.launch.py
```
There are 5 scenarios that you can select the scenario with the scenario parameter:
```bash
source install/setup.bash
ros2 launch acoustics_simulation_py acoustics_bringup.launch.py scenario:=1
```

***To run the simulation manually***

You need 13 terminals are needed.

**1.** 1 for the Micro XRCE DDS agent
```bash
MicroXRCEAgent udp4 -p 8888
```

**2.** 5 for the initialization of each of the 5 drones in gazebo
```bash
cd PX4-Autopilot && export PX4_HOME_LAT=43.13471 && export PX4_HOME_LON=6.01507 && export PX4_HOME_ALT=6 && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='5,0' PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1
```
(for example for the 1st drone)

**3.** 5 to run the offboard control for each drone to make it fly
```bash
source install/setup.bash
ros2 run swarmz_control_py offboard_control_py
```

**4.** 1 to run the acoustic simulation node to publish the sound topic of each drone in ROS2
```bash
source install/setup.bash
ros2 launch acoustics_simulation_py acoustics_bringup.launch.py
```

**5.** 1 to run the data plotter
```bash
source install/setup.bash
ros2 run plotjuggler plotjuggler
```


