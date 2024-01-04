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
        
        self.declare_parameter("d", 5)
        self.drones = self.get_parameter("d").get_parameter_value().integer_value

        self.declare_parameter("px4", '~/ros2_PX4_gz/PX4-Autopilot')
        self.px4 = self.get_parameter("px4").get_parameter_value().string_value
        
        self.declare_parameter("lat", 43.13471)
        self.latitude = self.get_parameter("lat").get_parameter_value().double_value

        self.declare_parameter("lon", 6.01507)
        self.longitude = self.get_parameter("lon").get_parameter_value().double_value

        self.declare_parameter("alt", 6)
        self.altitude = self.get_parameter("alt").get_parameter_value().double_value

        # List of commands to run
        commands = [
            # Run the Micro XRCE-DDS Agent
            "MicroXRCEAgent udp4 -p 8888"]

        # Spawn drones in PX4 SITL simulation
        for i in range(1,self.drones+1):
            commands.append("cd "+self.px4+" && export PX4_HOME_LAT="+str(self.latitude)+" && export PX4_HOME_LON="+str(self.longitude)+" && export PX4_HOME_ALT="+str(self.altitude)+" && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,"+str(i*5)+"' PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i "+str(i))

            # Run QGroundControl
            # "cd ~/QGroundControl && ./QGroundControl.AppImage

        # Kill old gazebo sim possibly running in the background
        try:
            subprocess.run(["pkill", "-f", 'gz sim'], check=True)
        except:
            pass
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", "pkill", "-f", "'gz", "sim'"])
        time.sleep(2)
        # Loop through each command in the list
        for command in commands:
            # Each command is run in a new tab of the gnome-terminal
            subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
            
            # Pause between each command
            time.sleep(2)

def main():
    rclpy.init()
    node = SimuNode()
    rclpy.spin(node)
    rclpy.shutdown()