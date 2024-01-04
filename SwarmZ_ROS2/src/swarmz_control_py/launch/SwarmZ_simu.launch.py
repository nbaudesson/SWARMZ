#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():
    #
    # ARGS
    #

    NB_OF_DRONES = 5
    
    headless = LaunchConfiguration("headless")
    headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo headless")

    px4_path = LaunchConfiguration("px4")
    px4_path_cmd = DeclareLaunchArgument(
        "px4",
        default_value="~/ros2_PX4_gz/PX4-Autopilot",
        description="PX4-Autopilot directory path")

    latitude = LaunchConfiguration("lat")
    latitude_cmd = DeclareLaunchArgument(
        "lat",
        default_value="43.13471",
        description="Latitude value for PX4 Home")
    
    longitude = LaunchConfiguration("lon")
    longitude_cmd = DeclareLaunchArgument(
        "lon",
        default_value="6.01507",
        description="Longitude value for PX4 Home")
    
    altitude = LaunchConfiguration("alt")
    altitude_cmd = DeclareLaunchArgument(
        "alt",
        default_value="6",
        description="Altitude value for PX4 Home")

    #
    # NODES
    #
    
    # simu_node=Node(
    #         package='swarmz_control',
    #         namespace='px4_offboard',
    #         executable='simu_processes',
    #         name='simu_processes',
    #         prefix='gnome-terminal --',
    #         parameters=[{"d": number_of_drones,
    #                      "px4": px4_path,
    #                      "lat": latitude,
    #                      "lon": longitude,
    #                      "alt": altitude}]
    #     )

    simu_node=Node(
        package='swarmz_control_py',
        namespace='px4_offboard',
        executable='simu_circle',
        name='simu_circle',
        # prefix='gnome-terminal --',
        parameters=[{"headless": headless,
                        "px4": px4_path,
                        "lat": latitude,
                        "lon": longitude,
                        "alt": altitude}]
    )

    offboard_control_nodes=[]
    for i in range(1, NB_OF_DRONES+1, 1):
        offboard_control_nodes.append(
            Node(
                package='swarmz_control_py',
                namespace='px4_'+str(i),
                executable='offboard_control_py',
                name='offboard_control_'+str(i),
                ))

    ld = LaunchDescription()

    ld.add_action(headless_cmd)
    ld.add_action(px4_path_cmd)
    ld.add_action(latitude_cmd)
    ld.add_action(longitude_cmd)
    ld.add_action(altitude_cmd)

    ld.add_action(simu_node)
    ld.add_action(TimerAction(period=37.0,actions=offboard_control_nodes))

    return ld
