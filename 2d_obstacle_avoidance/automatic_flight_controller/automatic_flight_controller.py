import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist

class AutomaticFlightController(Node):
    def __init__(self):
        super().__init__("automatic_flight_controller")
        self.lidar_subscriber = self.create_subscription(LaserScan, "/simple_drone/lidar",
                                                          self.flight_control, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, "/simple_drone/gps/nav",
                                                          self.flight_control, 10)
        self.target_subscriber = self.create_subscription(NavSatFix, "/simple_drone/target_location",
                                                          self.flight_control, 10)
        
        self.control_publisher = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)
        
    def flight_control(self):
        pass