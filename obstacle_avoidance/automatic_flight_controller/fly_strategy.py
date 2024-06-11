from ament_index_python.packages import get_package_share_directory
import os, yaml

from rclpy.node import Node
from automatic_flight_controller.direction_controller import DirectionController
from automatic_flight_controller.drone_data import DroneData
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from abc import abstractmethod

class FlyStrategy(Node):
    def __init__(self, drone_name):
        super().__init__(drone_name)
            
        self.__config_dict : dict = get_config(drone_name)
        self.__model_ns = self.__config_dict.get("namespace")
        self.drone_data = DroneData(self, self.__config_dict)
        self.__direction_controller = DirectionController(self.__config_dict, self.drone_data)
        self.__control_twist = Twist()
        
        # Publisher
        self.__control_publisher = self.create_publisher(Twist, "/{}/cmd_vel".format(self.__model_ns), 10)
        self.__target_direction_publish = self.create_publisher(Float32, "/{}/target_direction".format(self.__model_ns), 10)

        self.__timer = self.create_timer(0.001, self.__fly)

    def __fly(self):
        if (self.drone_data.target_received): 
            speed, target_direction = self.calculate_xy_velocity()
            self.__publish_target_direction(target_direction)
            self.__calculate_xy_control(target_direction, speed)
            self.__calculate_z_control()
            self.__publish_control()
            

    def __publish_target_direction(self, target_direction):
        self.__target_direction_publish.publish(Float32(data = target_direction))

    def __calculate_xy_control(self, target_direction, speed):
        self.__control_twist = self.__direction_controller.flight_control(target_direction, speed)

    def __calculate_z_control(self):
        self.__control_twist.linear.z = self.calculate_z_velocity()

    def __publish_control(self):
        self.__control_publisher.publish(self.__control_twist)

    @abstractmethod
    def calculate_xy_velocity(self):
        return (0.0, 0.0)
    
    @abstractmethod
    def calculate_z_velocity(self):
        return 0.0


def get_config(drone_name):
    yaml_path =  os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', drone_name + '.yaml')

    with open(yaml_path, 'r') as f:
        config_dict = yaml.load(f, Loader=yaml.FullLoader)

    return config_dict