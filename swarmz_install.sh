#!/bin/bash

# Following https://docs.px4.io/main/en/ros/ros2_comm.html

### ROS2 humble ###
while true; do
    echo "Do you use ROS2 Humble or ROS2 Foxy? (h/f): "
    read user_input

    if [ "$user_input" = "h" ]; then

        ### ROS2 humble ###
        while true; do
            echo "Do you want to install ROS2 Humble? (y/n): "
            read user_input
            ros_distro="humble"
            if [ "$user_input" = "y" ]; then
                echo "You chose to proceed."
                sudo apt update -y && sudo apt install locales -y
                sudo locale-gen en_US en_US.UTF-8
                sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
                export LANG=en_US.UTF-8
                sudo apt install software-properties-common -y
                sudo add-apt-repository universe
                sudo apt update -y && sudo apt install curl -y
                sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
                echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |
                sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
                sudo apt update -y && sudo apt upgrade -y
                sudo apt install ros-humble-desktop -y
                sudo apt install ros-dev-tools -y
                source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

                pip install --user -U empy pyros-genmsg setuptools
                break
            elif [ "$user_input" = "n" ]; then
                echo "You chose not to proceed."
                break
            else
                echo "Invalid input. Please enter 'y' or 'n'."
            fi
        done
        break
    elif [ "$user_input" = "f" ]; then
        echo "You chose Foxy."
        ros_distro="foxy"
        ### ROS2 humble ###
        while true; do
            echo "Do you want to install ROS2 Foxy? (y/n): "
            read user_input

            if [ "$user_input" = "y" ]; then
                echo "You chose to proceed."
                sudo apt update -y && sudo apt install locales -y
                sudo locale-gen en_US en_US.UTF-8
                sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
                export LANG=en_US.UTF-8
                sudo apt install software-properties-common -y
                sudo add-apt-repository universe
                sudo apt update -y && sudo apt install curl -y
                sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
                echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |
                sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
                sudo apt update -y && sudo apt upgrade -y
                sudo apt install ros-foxy-desktop -y
                sudo apt install ros-dev-tools -y
                source /opt/ros/foxy/setup.bash && echo "source /opt/ros/foxy/setup.bash" >> .bashrc

                pip install --user -U empy pyros-genmsg setuptools
                break
            elif [ "$user_input" = "n" ]; then
                echo "You chose not to proceed."
                break
            else
                echo "Invalid input. Please enter 'y' or 'n'."
            fi
        done
        break
    else
        echo "Invalid input. Please enter 'h' or 'f'."
    fi
done


### Gazebo ignition transport 11 ###
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update -y
sudo apt-get install libignition-transport11-dev -y

### ignition Gazebo 6 ###
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update -y

sudo apt-get install libignition-gazebo6-dev -y

### ros-$ros_distro-actuator-msgs ###
sudo apt install ros-$ros_distro-actuator-msgs -y

### ros-$ros_distro-plotjuggler-ros ###
sudo apt install ros-$ros_distro-plotjuggler-ros -y

### Gazebo garden (harmonic didn't work (yet)) ###
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update -y
sudo apt remove gz-classic -y ; sudo apt remove gz-garden -y ; sudo apt remove gz-harmonic -y ; sudo apt autoremove -y
# sudo apt-get install gz-garden -y
### Apparently it works now ?
if [ "$ros_distro" = "foxy" ]; then
sudo apt-get install gz-garden -y
fi
if [ "$ros_distro" = "humble" ]; then
sudo apt-get install gz-harmonic -y
fi
# sudo apt remove gz-harmonic -y ; sudo apt autoremove -y

### gflags (from binary) ###
sudo apt-get install libgflags-dev -y

# Default
### XRCE-DDS agent ### 
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make -j 4
sudo make install
sudo ldconfig /usr/local/lib/

### PX4 ###
cd ../..
cd PX4-Autopilot
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
python3 -m pip install -r ./PX4-Autopilot/Tools/setup/requirements.txt

make distclean
make px4_sitl -j 4

cd ..
cd SwarmZ_ROS2

cp map/default.sdf PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

### ros2 px4 offboard control ### github to pilot drone with velocity
# git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git

sudo rosdep init
rosdep update
rosdep install -r --from-paths src -i -y --rosdistro $ros_distro
source /opt/ros/$ros_distro/setup.bash

rm -rf build/ install/ log/
colcon build