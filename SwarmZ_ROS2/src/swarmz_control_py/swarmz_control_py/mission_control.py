import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

# MISSION={"1":[{"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 0, "y": 0, "z": None, "time":10},],
#         "2":[{"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 0, "y": 0, "z": None, "time":10},],
#         "3":[{"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 0, "y": 0, "z": None, "time":10},],
#         "4":[{"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 0, "y": 0, "z": None, "time":10},],
#         "5":[{"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 10, "y": 10, "z": None, "time":10},
#               {"x": 20, "y": 10, "z": None, "time":10},
#               {"x": 30, "y": 10, "z": None, "time":20},
#               {"x": 0, "y": 0, "z": None, "time":10},],}

# Au 4 coin
MISSION={
        "1":[{"x": 0,  "y": 0,  "z": None, "time":10},],
        "2":[{"x": 60, "y":-35, "z": None, "time":10},],
        "3":[{"x": 60, "y": 35, "z": None, "time":10},],
        "4":[{"x":-60, "y":-35, "z": None, "time":10},],
        "5":[{"x":-60, "y": 35, "z": None, "time":10},]
         }

# ligne /10m
MISSION={
        "1":[{"x": 0,  "y": 0, "z": None, "time":10},],
        "2":[{"x": 0, "y": 45, "z": None, "time":10},],
        "3":[{"x": 0, "y": -60, "z": None, "time":10},],
        "4":[{"x": 50, "y": -15, "z": None, "time":10},],
        "5":[{"x": -50, "y": -20, "z": None, "time":10},]
         }


MIN_HEIGHT=-2.0
MISSION_PRECISION = 0.15

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Get the namespace of the node
        self.node_namespace = self.get_namespace()
        self.get_logger().info('Extracted instance: %s' % self.node_namespace)

        # Extract the number at the end of the namespace
        last_slash = self.node_namespace.rfind('_')
        if last_slash != -1 and last_slash + 1 < len(self.node_namespace):
            instance_str = self.node_namespace[last_slash + 1:]
            try:
                # Convert the extracted string to an integer
                self.instance = int(instance_str)
                # Print the extracted number
                self.get_logger().info('Extracted instance # : %d' % self.instance)
            except ValueError as e:
                self.get_logger().error('Invalid instance format: %s' % str(e))
            except OverflowError as e:
                self.get_logger().error('instance out of range: %s' % str(e))
        else:
            self.get_logger().error('Unable to extract a instance from the namespace')
            self.instance = 0
            self.node_namespace = ""

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, self.node_namespace+'/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, self.node_namespace+'/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, self.node_namespace+'/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, self.node_namespace+'/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, self.node_namespace+'/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.following_setpoint = False
        self.last_setpoint = {"x":0.0, "y":0.0, "z":0.0}
        self.takeoff_height = -5.0
        self.landing_height = -1.0
        self.mission_timer = 0
        self.callback_timer = 0.1
        self.takeoff_flag = False

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.callback_timer, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def hover(self):
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if abs(self.last_setpoint["z"]) > abs(MIN_HEIGHT):
                # self.get_logger().info(f'Hover setpoints {[self.last_setpoint["x"], self.last_setpoint["y"], self.last_setpoint["z"]]}')
                self.publish_position_setpoint(self.last_setpoint["x"], self.last_setpoint["y"], self.last_setpoint["z"])
            else:
                # self.get_logger().info(f'Hover setpoints {[self.last_setpoint["x"], self.last_setpoint["y"], MIN_HEIGHT]}')
                self.publish_position_setpoint(self.last_setpoint["x"], self.last_setpoint["y"], MIN_HEIGHT)
                self.last_setpoint = {"x":self.last_setpoint["x"], "y":self.last_setpoint["y"], "z":MIN_HEIGHT}
    
    def isclose2obj(self, x: float, y: float, z: float):
        if abs(abs(self.vehicle_local_position.x) - abs(x)) <= MISSION_PRECISION and abs(abs(self.vehicle_local_position.y) - abs(y)) <= MISSION_PRECISION and abs(abs(self.vehicle_local_position.z) - abs(z)) <= MISSION_PRECISION:
            return True
        else:
            return False
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.following_setpoint = True
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1 + self.instance
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        self.mission_timer += self.callback_timer

        # Arm drone and takeoff
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.engage_offboard_mode()
            self.arm()
            self.takeoff_flag = True
            return

        # If the drone has not reach its last objective, it tries again
        if not(self.isclose2obj(self.last_setpoint['x'], self.last_setpoint['y'], self.last_setpoint['z'])):
            # self.get_logger().info(f"delta X =  {abs(abs(self.vehicle_local_position.x) - abs(self.last_setpoint['x']))}")
            # self.get_logger().info(f"delta Y =  {abs(abs(self.vehicle_local_position.y) - abs(self.last_setpoint['y']))}")
            # self.get_logger().info(f"delta Z =  {abs(abs(self.vehicle_local_position.z) - abs(self.last_setpoint['z']))}")
            self.hover()
            return

        # Takeoff
        if self.takeoff_flag and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.last_setpoint = {"x":0.0, "y":0.0, "z":self.takeoff_height}
            self.takeoff_flag = False
            return

        # If no new setpoint is given and drone is at last setpoint then hover over last setpoint
        if MISSION[str(self.instance)] and not self.takeoff_flag:

            # Check format of mission coordinates
            if "x" in MISSION[str(self.instance)][0] and "y" in MISSION[str(self.instance)][0] and "z" in MISSION[str(self.instance)][0]:
                if MISSION[str(self.instance)][0]["z"] == None:
                    MISSION[str(self.instance)][0]["z"] = MIN_HEIGHT

                if self.isclose2obj(MISSION[str(self.instance)][0]["x"], MISSION[str(self.instance)][0]["y"], MISSION[str(self.instance)][0]["z"]):
                    if self.mission_timer >= MISSION[str(self.instance)][0]["time"]:
                        self.get_logger().info(f'Point {[MISSION[str(self.instance)][0]["x"], MISSION[str(self.instance)][0]["y"], MISSION[str(self.instance)][0]["z"]]} reached')
                        MISSION[str(self.instance)].pop(0)

                # Give new mission objective
                else:
                    if abs(self.vehicle_local_position.z) >= abs(MIN_HEIGHT) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                        self.get_logger().info(f'Publishing mission setpoints {[MISSION[str(self.instance)][0]["x"], MISSION[str(self.instance)][0]["y"], MISSION[str(self.instance)][0]["z"]]}')
                        self.publish_position_setpoint(float(MISSION[str(self.instance)][0]["x"]), float(MISSION[str(self.instance)][0]["y"]), float(MISSION[str(self.instance)][0]["z"]))
                        self.last_setpoint = {"x":float(MISSION[str(self.instance)][0]["x"]), "y":float(MISSION[str(self.instance)][0]["y"]), "z":float(MISSION[str(self.instance)][0]["z"])}
                        self.mission_timer = 0
            else:
                self.get_logger().info(f"Mission setpoints format incorrect")
        else:
            self.hover()

        # If after takeoff drone is below minimal height, it lands
        if abs(self.vehicle_local_position.z) <= abs(self.landing_height) and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and not self.takeoff_flag:
            self.land()
            self.get_logger().info(f"Landing")
            # exit(0)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
