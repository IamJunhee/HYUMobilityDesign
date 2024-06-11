from automatic_flight_controller.fly_strategy import FlyStrategy
import numpy as np
import math
import rclpy

# 충분한 높이까지 상승하여 직진으로 비행하는 방법
class DirectFly(FlyStrategy):
    def __init__(self, drone_name, target_altitude):
        super().__init__(drone_name)
        self.get_logger().info("Direct Fly Strategy is started")
        self.__is_at_target_altitude = False
        self.__target_altitude = target_altitude

    def calculate_xy_velocity(self):
        if self.__is_at_target_altitude:
            direction_arr = np.array((
                -(self.drone_data.target.longitude - self.drone_data.gps.longitude),
                -(self.drone_data.target.latitude - self.drone_data.gps.latitude)
            ))

            direction_arr = direction_arr / np.linalg.norm(direction_arr, 2)

            temp_target_direction = math.atan2(direction_arr[1], direction_arr[0])

            if temp_target_direction < 0:
                return 15, 2 * math.pi + temp_target_direction
            else:
                return 15, temp_target_direction # 0 ~ 2pi
        else:
            return 0, self.drone_data.heading_direction # Not Move
        
    def calculate_z_velocity(self):
        err = self.__target_altitude - self.drone_data.gps.altitude
        if abs(err) < 0.1:
            self.__is_at_target_altitude == True

        return err

        

def main(args=None):
    rclpy.init(args=args)

    flight_strategy = DirectFly("drone", 60.0)

    rclpy.spin(flight_strategy)
    flight_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()