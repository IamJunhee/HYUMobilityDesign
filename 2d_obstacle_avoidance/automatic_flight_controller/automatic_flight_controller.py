import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix, Imu
from geometry_msgs.msg import Twist, Vector3, Quaternion
import numpy as np
import math

class AutomaticFlightController(Node):
    def __init__(self):
        super().__init__("automatic_flight_controller")
        
        #Subscriptiself.__directionon
        self.lidar_subscriber = self.create_subscription(LaserScan, "/simple_drone/lidar",
                                                          self.__set_lidar, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, "/simple_drone/gps/nav",
                                                          self.__set_gps, 10)
        self.target_subscriber = self.create_subscription(NavSatFix, "/simple_drone/target_location",
                                                          self.__set_target, 10)
        self.imu_subscriber = self.create_subscription(Imu, "/simple_drone/imu/out",
                                                        self.__set_angle, 10)
        
        # Publisher
        self.control_publisher = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)
        self.direction_publisher = self.create_publisher(Vector3, "/simple_drone/direction", 10)

        # Control Info
        self.__lidar : LaserScan = LaserScan()
        self.__gps : NavSatFix = NavSatFix()
        self.__target : NavSatFix = NavSatFix()
        self.__current_direction : Vector3 = Vector3()
        self.__direction : Vector3 = Vector3()


    def __set_lidar(self, msg):
        self.__lidar = msg
        self.flight_control()

    def __set_gps(self, msg):
        self.__gps = msg
        self.flight_control()

    def __set_target(self, msg : NavSatFix):
        self.get_logger().info("Set new target lat : %f  long : %f  alt : %f" % (msg.latitude, msg.longitude, msg.altitude))
        self.__target = msg
        self.flight_control()

    def __set_angle(self, msg: Imu):
        quaternion : Quaternion = msg.orientation
        (self.__current_direction.x, self.__current_direction.y, self.__current_direction.z) \
            = euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.get_logger().info("Current angle x : %f y : %f z : %f" % (self.__current_direction.x, self.__current_direction.y, self.__current_direction.z))
        self.flight_control()

        
    def flight_control(self):
        if self.is_arrival(0.00001):
            self.publish_stop()
        else:
            self.calculate_direction()
            self.publish_control()
    
    def calculate_distance(self):
        target = np.array((self.__target.longitude, self.__target.latitude, 0))
        current = np.array((self.__gps.longitude, self.__gps.latitude, 0))

        return np.linalg.norm(target - current, 2)

    def is_arrival(self, delta):
        distance = self.calculate_distance()

        self.get_logger().info("distance to target : %f" % distance)

        return distance < delta

    def calculate_direction(self):
        direction_arr = np.array((
            -(self.__target.latitude - self.__gps.latitude),
            -(self.__target.longitude - self.__gps.longitude),
            0.0
        ))

        direction_arr = direction_arr / np.linalg.norm(direction_arr, 2)

        self.__direction.x = direction_arr[0]
        self.__direction.y = direction_arr[1]
        self.__direction.z = direction_arr[2]

        self.direction_publisher.publish(self.__direction)

        self.get_logger().info("current %f, target %f" % (self.__current_direction.z ,math.atan2(self.__direction.y, self.__direction.x)))
        

    def publish_control(self):
        k_p = 2.0
        angular_vel = Vector3(
            x = 0.0,
            y = 0.0,
            z = k_p * (math.atan2(self.__direction.y, self.__direction.x) - self.__current_direction.z)
        )
        
        self.control_publisher.publish(
            Twist(linear = Vector3(x = 5.0, y = 0.0, z = 0.0),
                  angular = angular_vel)
        )

    def publish_stop(self):
        self.control_publisher.publish(
            Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        )

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

def main(args=None):
    rclpy.init(args=args)

    controller = AutomaticFlightController()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()