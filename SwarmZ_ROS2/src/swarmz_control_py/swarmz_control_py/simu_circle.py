# Import the subprocess and time modules
import rclpy
from rclpy.node import Node
import subprocess
import time
import sys
import argparse

class SimuNode(Node):

    def __init__(self) -> None:
        super().__init__("simu_node")

        self.declare_parameter("headless", 0)
        self.headless = self.get_parameter("headless").get_parameter_value().integer_value

        self.declare_parameter("px4", '../PX4-Autopilot')
        self.px4 = self.get_parameter("px4").get_parameter_value().string_value
        
        self.declare_parameter("lat", 43.13471)
        self.latitude = self.get_parameter("lat").get_parameter_value().double_value

        self.declare_parameter("lon", 6.01507)
        self.longitude = self.get_parameter("lon").get_parameter_value().double_value

        self.declare_parameter("alt", 6)
        self.altitude = self.get_parameter("alt").get_parameter_value().double_value

        if self.headless == 0:
            self.gui=""
        else:
            self.gui="HEADLESS=1"

def main():
    rclpy.init()
    node = SimuNode()

    # List of commands to run
    commands = [
        # Run the Micro XRCE-DDS Agent
        "MicroXRCEAgent udp4 -p 8888",
    
        # Spawn drones in PX4 SITL simulation
        "cd "+node.px4+" && export PX4_HOME_LAT="+str(node.latitude)+" && export PX4_HOME_LON="+str(node.longitude)+" && export PX4_HOME_ALT="+str(node.altitude)+" && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='5,0' PX4_GZ_MODEL=x500 "+node.gui+" ./build/px4_sitl_default/bin/px4 -i 1",
        "cd "+node.px4+" && export PX4_HOME_LAT="+str(node.latitude)+" && export PX4_HOME_LON="+str(node.longitude)+" && export PX4_HOME_ALT="+str(node.altitude)+" && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='1.6,4.7' PX4_GZ_MODEL=x500 "+node.gui+" ./build/px4_sitl_default/bin/px4 -i 2",
        "cd "+node.px4+" && export PX4_HOME_LAT="+str(node.latitude)+" && export PX4_HOME_LON="+str(node.longitude)+" && export PX4_HOME_ALT="+str(node.altitude)+" && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='-4,3' PX4_GZ_MODEL=x500 "+node.gui+" ./build/px4_sitl_default/bin/px4 -i 3",
        "cd "+node.px4+" && export PX4_HOME_LAT="+str(node.latitude)+" && export PX4_HOME_LON="+str(node.longitude)+" && export PX4_HOME_ALT="+str(node.altitude)+" && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='-4,-3' PX4_GZ_MODEL=x500 "+node.gui+" ./build/px4_sitl_default/bin/px4 -i 4",
        "cd "+node.px4+" && export PX4_HOME_LAT="+str(node.latitude)+" && export PX4_HOME_LON="+str(node.longitude)+" && export PX4_HOME_ALT="+str(node.altitude)+" && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='1.6,-4.7' PX4_GZ_MODEL=x500 "+node.gui+" ./build/px4_sitl_default/bin/px4 -i 5",

        # Run QGroundControl
        # "cd ~/QGroundControl && ./QGroundControl.AppImage
    ]

    # Kill old gazebo sim possibly running in the background
    command = "pkill -f 'gz sim'"
    try:
        subprocess.run(command, shell=True, check=True)
        print("Command executed successfully")
    except subprocess.CalledProcessError as e:
        # Handle the error, or ignore it if it's due to no matching processes
        if e.returncode != 1:  # Check if the return code is not 1 (no processes matched)
            print(f"Error executing command: {e}")
        else:
            print("No processes matched the pattern.")
    time.sleep(2)

    # Loop through each command in the list
    for command in commands:
        # Each command is run in a new tab of the gnome-terminal
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
        
        # Pause between each command
        time.sleep(2.5)
