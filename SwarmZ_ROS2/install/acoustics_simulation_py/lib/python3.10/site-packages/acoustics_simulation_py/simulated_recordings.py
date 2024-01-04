import rclpy
import functools
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3
from px4_msgs.msg import SensorGps, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import copy
import yaml

SPEED_OF_SOUND = 343  # m/s
PI = 3.1415

class GenerateSounds(Node):
    def __init__(self):
        super().__init__('generate_sounds')

        self.declare_parameter("nb_of_drones", 5)
        self.nb_of_drones= self.get_parameter("nb_of_drones").get_parameter_value().integer_value

        self.declare_parameter("scenario", "src/accoustics_simulation_py/config/scenarios/scenario_2.yaml")
        self.scenario = self.get_parameter("scenario").get_parameter_value().string_value        
        with open(self.scenario, 'r') as file:
            self.EXPLOSIONS = yaml.safe_load(file)

        self.declare_parameter("noises", "src/accoustics_simulation_py/config/sounds/noises.yaml")
        self.noises = self.get_parameter("noises").get_parameter_value().string_value        
        with open(self.noises, 'r') as file:
            self.NOISES = yaml.safe_load(file)

        self.timer = 0.001
        self.decimal = str(self.timer)[::-1].find('.')
        self.max_index = max(range(len(self.EXPLOSIONS)), key=lambda i: self.EXPLOSIONS[i]["start_time"]+self.EXPLOSIONS[i]["lenght"])

        self.count_time = 0
        self.sub_pos_gps = {}
        self.pos_gps = {}
        self.source_pub = {}
        self.white_noise = 0

        # Configure QoS profile for publishing and subscribing
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        for i in range(1, self.nb_of_drones +1, 1):
            # Get the GPS location of the drone
            self.sub_pos_gps["sub_pos_gps_r%d" % i] = self.create_subscription(SensorGps, "/px4_%d/fmu/out/vehicle_gps_position" % i, functools.partial(self.pos_gps_callback, topic_num = i), qos)

            # Publishers for the simulated sounds preceived by the microphones of the drones
            self.source_pub["source_0%d_publisher" % i] = self.create_publisher(Float32, "/px4_%d/fmu/in/mono_sound_sensor" % i, 10)

        self.pub_timer = self.create_timer(self.timer, self.timer_callback)

    def pos_gps_callback(self, msg, topic_num = 0):
        self.pos_gps[topic_num] = {"lat": msg.latitude_deg, "lon": msg.longitude_deg, "alt": msg.altitude_msl_m}

    def timer_callback(self):
        # Wait for drones to publish their gps position
        if len(self.pos_gps) < self.nb_of_drones:
            return
        
        # Initiate variables
        self.count_time = round(self.count_time + self.timer, self.decimal)
        drone_distance_to_drones = {}
        motor_noise_from_drones = {}
        for i in range(1, self.nb_of_drones + 1):
            inner_dictionary = {key: 0 for key in range(1, self.nb_of_drones + 1)}
            drone_distance_to_drones[i] = inner_dictionary

        motor_noise_from_drones = copy.deepcopy(drone_distance_to_drones)

        for e in range(len(self.EXPLOSIONS)):
            if self.EXPLOSIONS[e]["start_time"] == self.count_time:
                print("!!!!!!!!!!!!!!")
                print("!!!! BOOM !!!! : ", e)
                print("!!!!!!!!!!!!!!")

        # Calculate each sound perceived by each drone
        for i in range(1, self.nb_of_drones +1, 1):
            simulated_noise=0

            # Calculate background noise noise perceived by drone i
            self.white_noise = self.calculate_source_motor(self.NOISES["white_noise"]["amplitude_1"], self.NOISES["white_noise"]["amplitude_3"],
                                                           self.NOISES["white_noise"]["phase_1"], self.NOISES["white_noise"]["phase_3"],
                                                           self.NOISES["white_noise"]["frequency"], 1)
            simulated_noise += self.white_noise

            # Calculate noise from each explosion perceived by drone i
            explosions_noise = 0
            for e in range(len(self.EXPLOSIONS)):
                if ((self.count_time >= self.EXPLOSIONS[e]["start_time"]) and (self.count_time <= (self.EXPLOSIONS[e]["start_time"] + self.EXPLOSIONS[e]["lenght"]))):
                    count_time_explosion = self.count_time - self.EXPLOSIONS[e]["start_time"]

                    # Calculate the distance between the drones and the explosion
                    drone_distance_to_explosion = self.calculate_distance(self.pos_gps[i]["lat"], self.pos_gps[i]["lon"], self.pos_gps[i]["alt"],
                                                                        self.EXPLOSIONS[e]["lat"], self.EXPLOSIONS[e]["lon"], self.EXPLOSIONS[e]["alt"])
                    
                    if count_time_explosion % int(self.EXPLOSIONS[e]["lenght"]/ 2) == 0:
                        print("d",i,"to e",e,"=", round(drone_distance_to_explosion, 2))

                    # Hoang-Anh need to explain this !
                    # if (self.EXPLOSIONS[e]["lenght"]/count_time_explosion >= drone_distance_to_explosion/SPEED_OF_SOUND):
                    # if (count_time_explosion/10 >= drone_distance_to_explosion/SPEED_OF_SOUND):

                    explosion_noise = self.calculate_source_explosion(self.NOISES["explosion_noise"]["amplitude_1"], self.NOISES["explosion_noise"]["amplitude_3"],
                                                                      self.NOISES["explosion_noise"]["phase_1"], self.NOISES["explosion_noise"]["phase_3"],
                                                                      self.NOISES["explosion_noise"]["frequency"], drone_distance_to_explosion,
                                                                      count_time_explosion, self.EXPLOSIONS[e]["lenght"])

                    explosions_noise += explosion_noise

            simulated_noise += explosions_noise

            # Calculate noise from other drones perceived by drone i
            for j in range(1, self.nb_of_drones +1, 1):
                # distance to self case
                if i == j:
                    drone_distance_to_drones[i][j] = 1
                    motor_noise_from_drones[i][j] = self.calculate_source_motor(self.NOISES["motor_noise"]["amplitude_1"], self.NOISES["motor_noise"]["amplitude_3"],
                                                                                self.NOISES["motor_noise"]["phase_1"], self.NOISES["motor_noise"]["phase_3"],
                                                                                self.NOISES["motor_noise"]["frequency"], drone_distance_to_drones[i][j])
                    motor_noise_from_drones[i][j] = 0
                    
                # If calculus has been made in one way, copy the result for the other way
                elif drone_distance_to_drones[j][i]!=0:
                    drone_distance_to_drones[i][j] = drone_distance_to_drones[j][i]
                    motor_noise_from_drones[i][j] = motor_noise_from_drones[j][i]

                # Calculate the distance between the drones
                else:
                    drone_distance_to_drones[i][j] = self.calculate_distance(self.pos_gps[i]["lat"], self.pos_gps[i]["lon"], self.pos_gps[i]["alt"],
                                                                         self.pos_gps[j]["lat"], self.pos_gps[j]["lon"], self.pos_gps[j]["alt"])

                    
                    motor_noise_from_drones[i][j] = self.calculate_source_motor(self.NOISES["motor_noise"]["amplitude_1"], self.NOISES["motor_noise"]["amplitude_3"],
                                                                                self.NOISES["motor_noise"]["phase_1"], self.NOISES["motor_noise"]["phase_3"],
                                                                                self.NOISES["motor_noise"]["frequency"], drone_distance_to_drones[i][j])

                simulated_noise += motor_noise_from_drones[i][j]

            # Publish the results
            self.source_pub["source_0%d_publisher" % i].publish(Float32(data=simulated_noise))

        # Print every 5 seconds
        if self.count_time % 5 == 0:
            rounded_distances = {
                drone: {key: "{:.2f}".format(round(value, 2)) for key, value in distance.items()}
                for drone, distance in drone_distance_to_drones.items()}
            rounded_noises = {
                drone: {key: "{:.2f}".format(round(value, 2)) for key, value in noise.items()}
                for drone, noise in motor_noise_from_drones.items()}
            print("Distances")
            for drone, distance in rounded_distances.items():
                print(str(drone) + ": " + str(distance))
            print("Noises")
            for drone, noise in rounded_noises.items():
                print(str(drone) + ": " + str(noise))
            print()

        # Timer reset time, 2 min after last interesting thing happened
        if len(self.EXPLOSIONS)>0:
            if self.EXPLOSIONS[self.max_index]["start_time"] + self.EXPLOSIONS[self.max_index]["lenght"] + 2 * 60 <=  self.count_time: 
                self.count_time = 0

    def calculate_source_explosion(self, amplitude_source_1, amplitude_source_3,
                                   phase_source_1, phase_source_3,
                                   frequency, dist_source2robot,
                                   count_time_explosion, explosion_lenght):
        delta_phase_1 = 2 * PI * dist_source2robot * frequency / SPEED_OF_SOUND
        delta_phase_3 = 2 * PI * dist_source2robot * 3 * frequency / SPEED_OF_SOUND

        return (1 / (dist_source2robot * dist_source2robot) * math.exp(-count_time_explosion / explosion_lenght)
            * self.generate_sinwave(amplitude_source_1, frequency, phase_source_1, delta_phase_1, amplitude_source_3, frequency, phase_source_3, delta_phase_3, self.count_time)
               )

    def calculate_source_motor(self, amplitude_source_1, amplitude_source_3,
                               phase_source_1, phase_source_3,
                               frequency, dist_source2robot):
        delta_phase_1 = 2 * PI * dist_source2robot * frequency / SPEED_OF_SOUND
        delta_phase_3 = 2 * PI * dist_source2robot * 3 * frequency / SPEED_OF_SOUND

        return ((1 / (dist_source2robot * dist_source2robot)) 
            * self.generate_sinwave(amplitude_source_1, frequency, phase_source_1, delta_phase_1, amplitude_source_3, frequency, phase_source_3, delta_phase_3, self.count_time)
               )

    def generate_sinwave(self, a1, f1, p1, d1, a3, f3, p3, d3, t):
        # Amplitude*sin(2*pi*frequency*time+phase+deltaphase)
        return (a1 * math.sin(2 * PI * f1 * t + p1 + d1)  # Signal
              + a3 * math.sin(2 * PI * f3 * t + p3 + d3)) # Harmonics

    def calculate_distance(self, lat1, lon1, alt1, lat2, lon2, alt2):
        # Function to calculate the distance using Vincenty formula
        # Convert latitude and longitude from degrees to radians
        lat1 = lat1 * math.pi / 180.0
        lon1 = lon1 * math.pi / 180.0
        lat2 = lat2 * math.pi / 180.0
        lon2 = lon2 * math.pi / 180.0

        # Earth radius in meters
        R = 6371009

        # Calculate the differences
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        dalt = alt2 - alt1

        # Vincenty formula for distance calculation
        a = (math.sin(dlat / 2)) ** 2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon / 2))**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Calculate the distance
        distance = R * c
        
        # Correct for altitude
        distance = math.sqrt(distance * distance + dalt ** 2)

        return distance


def main(args=None):
    rclpy.init(args=args)

    generate_sounds = GenerateSounds()

    rclpy.spin(generate_sounds)

    generate_sounds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
