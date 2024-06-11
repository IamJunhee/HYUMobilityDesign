from rclpy.node import Node

from sensor_msgs.msg import LaserScan, NavSatFix, Imu
from geometry_msgs.msg import Twist, Vector3, Quaternion

import math

class DroneData:
    def __init__(self, node: Node, config_dict: dict):
        self.__node : Node = node
        self.__config : dict = config_dict
        self.__model_ns = self.__config.get("namespace", "simple_drone")
       
        #Subscription
        self.__lidar_subscriber = self.__node.create_subscription(LaserScan, "/{}/lidar".format(self.__model_ns),
                                                          self.__set_lidar, 10)
        self.__gps_subscriber = self.__node.create_subscription(NavSatFix, "/{}/gps/nav".format(self.__model_ns),
                                                          self.__set_gps, 10)
        self.__target_subscriber = self.__node.create_subscription(NavSatFix, "/{}/target_location".format(self.__model_ns),
                                                          self.__set_target, 10)
        self.__imu_subscriber = self.__node.create_subscription(Imu, "/{}/imu/out".format(self.__model_ns),
                                                        self.__set_angle, 10)
        
        self.gps = NavSatFix()
        self.target = NavSatFix()
        self.heading_direction = 0.0
        self.lidar =LaserScan()
        self.target_received = False
        
        
    def __set_lidar(self, msg: LaserScan):
        self.lidar = msg

    def __set_gps(self, msg: NavSatFix):
        self.gps.latitude = msg.latitude * (10 ** 5)
        self.gps.longitude =  msg.longitude * (10 ** 5)
        self.gps.altitude = msg.altitude

    def __set_target(self, msg : NavSatFix):
        self.target.latitude = msg.latitude
        self.target.longitude =  msg.longitude
        self.target.altitude = msg.altitude
        self.target_received = True

    def __set_angle(self, msg: Imu):
        quaternion : Quaternion = msg.orientation
        (__x, __y, z) \
            = euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if z < 0:
            self.heading_direction = 2 * math.pi + z
        else:
            self.heading_direction = z # 0 ~ 2pi

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians