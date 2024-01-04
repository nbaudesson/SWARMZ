#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #
    # ARGS
    #
    
    offboard_control_1_node=Node(
            package='px4_ros_com',
            namespace='px4_1',
            executable='offboard_control',
            name='offboard_control_1',
            remappings=[
                ('/fmu/in/offboard_control_mode', 'fmu/in/offboard_control_mode'),
                ('/fmu/in/trajectory_setpoint', 'fmu/in/trajectory_setpoint'),
                ('/fmu/in/vehicle_command', 'fmu/in/vehicle_command'),
            ]
        )

    ld = LaunchDescription()

    ld.add_action(offboard_control_1_node)

    return ld
