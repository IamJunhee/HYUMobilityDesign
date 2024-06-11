from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import math
from automatic_flight_controller.drone_data import DroneData


class DirectionController:
    def __init__(self, node: Node, config_dict: dict, data: DroneData):
        self.__node = node
        self.__config : dict = config_dict
        self.__model_ns = self.__config.get("namespace", "simple_drone")
        self.__data = data
        self.__target_direction : float = 0.0
        
        # Publisher
        self.control_publisher = self.__node.create_publisher(Twist, "/{}/cmd_vel".format(self.__model_ns), 10)

    def flight_control(self, target_direction : float):
        self.__target_direction = target_direction

        if self.__is_arrival():
            self.__publish_stop()
        else:
            self.__publish_control()

    def __is_arrival(self):
        arrival_range = self.__config.get("arrivalRange", 1)

        distance = calculate_distance(self.__data.target, self.__data.gps)
        self.__node.get_logger().info("distance to target : %f" % distance)

        return distance < arrival_range
        
    def __publish_control(self):
        k_p = self.__config.get("directionProportionalGain", 2.0)
        speed = self.__config.get("speed", 10)

        direction_error = calculate_direction_err(self.__target_direction, self.__data.heading_direction)
        self.__node.get_logger().info("err : %f" % direction_error)

        angular_vel = Vector3(
            x = 0.0,
            y = 0.0,
            z = (k_p * direction_error)
        )

        linear_vel = Vector3(
            x = speed * math.cos(direction_error),
            y = speed * math.sin(direction_error),
            z = 0.0
        )
        
        self.control_publisher.publish(
            Twist(linear = linear_vel,
                  angular = angular_vel)
        )

    def __publish_stop(self):
        self.__data.target_received = False
        self.control_publisher.publish(
            Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        )

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