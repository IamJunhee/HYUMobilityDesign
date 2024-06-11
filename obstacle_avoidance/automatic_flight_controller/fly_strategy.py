from ament_index_python.packages import get_package_share_directory
import os, yaml

from rclpy.node import Node
from automatic_flight_controller.direction_controller import DirectionController
from automatic_flight_controller.drone_data import DroneData
from std_msgs.msg import Float32

from abc import abstractmethod

class FlyStrategy(Node):
    def __init__(self, drone_name):
        super().__init__(drone_name)
            
        self.__config_dict : dict = get_config(drone_name)
        self.__model_ns = self.__config_dict.get("namespace")
        self.drone_data = DroneData(self, self.__config_dict)
        self.__controller = DirectionController(self, self.__config_dict, self.drone_data)
        
        self.__target_direction_publish = self.create_publisher(Float32, "/{}/target_direction".format(self.__model_ns), 10)

        self.__timer = self.create_timer(0.001, self.__fly)

    def __fly(self):
        if (self.drone_data.target_received): 
            target_direction = self.calculate_direction()
            self.__publish_target_direction(target_direction)
            self.__controller.flight_control(target_direction)

    def __publish_target_direction(self, target_direction):
        self.__target_direction_publish.publish(Float32(data = target_direction))

    @abstractmethod
    def calculate_direction(self):
        return 0.0


def get_config(drone_name):
    yaml_path =  os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', drone_name + '.yaml')

    with open(yaml_path, 'r') as f:
        config_dict = yaml.load(f, Loader=yaml.FullLoader)

    return config_dict