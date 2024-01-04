from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    #
    # ARGS
    #
    scenario = LaunchConfiguration("scenario")
    scenario_cmd = DeclareLaunchArgument(
        "scenario",
        default_value="src/accoustics_simulation_py/scenarios/test_scenario.yaml",
        description="scenario path")

    nb_of_drones = LaunchConfiguration("nb_of_drones")
    nb_of_drones_cmd = DeclareLaunchArgument(
        "nb_of_drones",
        default_value="5",
        description="nb_of_drones in pixels")


    #
    # NODES
    #
    simulated_noises_cmd = Node(
        package="acoustics_simulation",
        executable="simulated_noises",
        name="simulated_noises_py",
        output="screen",
        parameters=[{"scenario": scenario,
                     "nb_of_drones": nb_of_drones,
                   }],
    )

    ld = LaunchDescription()

    ld.add_action(scenario_cmd)
    ld.add_action(nb_of_drones_cmd)

    ld.add_action(simulated_noises_cmd)

    return ld
