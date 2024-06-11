from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import math
from automatic_flight_controller.drone_data import DroneData


class DirectionController:
    def __init__(self, config_dict: dict, data: DroneData):
        self._config : dict = config_dict
        self.__data = data
        self.__target_direction : float = 0.0
        self.__speed  = 0.0

    def flight_control(self, target_direction : float, speed : float):
        self.__target_direction = target_direction
        self.__speed = speed

        return self.__calculate_control()

        
    def __calculate_control(self):
        if not(self.__is_arrival()):
            k_p = self._config.get("directionProportionalGain", 2.0)

            direction_error = calculate_direction_err(self.__target_direction, self.__data.heading_direction)

            angular_vel = Vector3(
                x = 0.0,
                y = 0.0,
                z = (k_p * direction_error)
            )

            linear_vel = Vector3(
                x = self.__speed * math.cos(direction_error),
                y = self.__speed * math.sin(direction_error),
                z = 0.0
            )
            
            return Twist(linear = linear_vel,
                        angular = angular_vel)
        
        else: 
            return Twist()
    
    
    def __is_arrival(self):
        arrival_range = self._config.get("arrivalRange", 1)

        distance = calculate_distance(self.__data.target, self.__data.gps)

        return distance < arrival_range


def calculate_direction_err(target, current):
    err = target - current
    if err > math.pi:
        err -= 2 * math.pi
    elif err < -math.pi:
        err += 2 * math.pi

    return err

def calculate_distance(target_gps: NavSatFix, current_gps: NavSatFix):
        target = np.array((target_gps.longitude, target_gps.latitude))
        current = np.array((current_gps.longitude, current_gps.latitude))

        return np.linalg.norm(target - current, 2)