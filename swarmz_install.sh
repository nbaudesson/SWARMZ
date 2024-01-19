#!/bin/bash
sudo apt install -y build-essential
# You need at least python 3.10 for numpy dependencies
desired_python_version="3.10"
# Get the installed Python version
installed_python_version=$(python3 --version 2>&1 | awk '{print $2}')
# Compare the versions
if [[ "$(printf "%s\n" "$desired_python_version" "$installed_python_version" | sort -V | head -n 1)" == "$desired_python_version" ]]; then
    echo "Python $desired_python_version or above is already installed."
else
    echo "Installing Python 3.12..."
    sudo apt-get purge --auto-remove python$installed_python_version -y
    sudo apt update
    sudo apt install software-properties-common -y
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt update
    sudo apt install python3.12 -y
    sudo apt install python-is-python3 -y
    sudo apt-get install curl -y
    curl -sS https://bootstrap.pypa.io/get-pip.py | python3.12 
fi
# Check if the system is running Ubuntu
if [ -f /etc/os-release ]; then
    # Source the os-release file
    . /etc/os-release

    # Check if it is Ubuntu and the version is either 20.04 or 22.04
    if [ "$ID" = "ubuntu" ] && ( [ "$VERSION_ID" = "20.04" ] || [ "$VERSION_ID" = "22.04" ] ); then
        echo "The system is running Ubuntu $VERSION_ID"

        if [ "$VERSION_ID" = "20.04" ]; then
            sudo apt update
            sudo apt install -y wget
            wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg
            echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list > /dev/null
            sudo apt update
            sudo apt install -y cmake
        fi
        # Check for ROS installation directories based on Ubuntu version
        if [ "$VERSION_ID" = "20.04" ] && [ -d "/opt/ros/foxy" ]; then
            echo "ROS Foxy is installed in /opt/ros/foxy."
            ros_distro="foxy"

        elif [ "$VERSION_ID" = "22.04" ] && [ -d "/opt/ros/humble" ]; then
            echo "ROS Humble is installed in /opt/ros/humble."
            ros_distro="humble"

        else
            echo "ROS installation directory not found for the specified Ubuntu version."
            if [ "$VERSION_ID" = "20.04" ] && [ ! -d "/opt/ros/foxy" ]; then
                echo "Installing ROS Foxy"
                ros_distro="foxy"
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
                source /opt/ros/foxy/setup.bash && echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

                sudo apt install python3-pip -y
                pip install --user -U empy pyros-genmsg
                pip install --user setuptools==58.2.0
                pip install empy==3.3.4
                pip install -U colcon-common-extensions

            elif [ "$VERSION_ID" = "22.04" ] && [ ! -d "/opt/ros/humble" ]; then
                echo "Installing ROS Humble"
                ros_distro="humble"
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
                source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

                sudo apt install python3-pip -y
                pip install --user -U empy pyros-genmsg
                pip install --user setuptools==58.2.0
                pip install empy==3.3.4
                pip install -U colcon-common-extensions

            fi
        fi
    else
        echo "This script is designed for Ubuntu 20.04 or 22.04, but the system does not match the criteria."
        
    fi
else
    echo "Unable to determine the operating system."

fi

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
echo "ros_distro" = "$ros_distro"
sudo apt-get install gz-garden -y
fi
if [ "$ros_distro" = "humble" ]; then
echo "ros_distro" = "$ros_distro"
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
make
sudo make install
sudo ldconfig /usr/local/lib/

### PX4 ###
cd ../..
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
python3 -m pip install -r ./Tools/setup/requirements.txt

make distclean
make px4_sitl

cd ..

### custom map ### 124x72m
cp map/swarmzmap.sdf PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

### buld ROS2 workspace ###
cd SwarmZ_ROS2


### ros2 px4 offboard control ### github to pilot drone with velocity
# git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git

sudo rosdep init
rosdep update
rosdep install -r --from-paths src -i -y --rosdistro $ros_distro
source /opt/ros/$ros_distro/setup.bash

rm -rf build/ install/ log/
colcon build