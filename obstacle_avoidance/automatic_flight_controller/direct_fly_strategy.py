from automatic_flight_controller.fly_strategy import FlyStrategy
import numpy as np
import math
import rclpy

class DirectFly(FlyStrategy):
    def __init__(self, drone_name):
        super().__init__(drone_name)
        self.get_logger().info("Direct Fly Strategy is started")

    def calculate_direction(self):
        direction_arr = np.array((
            -(self.drone_data.target.longitude - self.drone_data.gps.longitude),
            -(self.drone_data.target.latitude - self.drone_data.gps.latitude)
        ))

        direction_arr = direction_arr / np.linalg.norm(direction_arr, 2)

        temp_target_direction = math.atan2(direction_arr[1], direction_arr[0])

        if temp_target_direction < 0:
            return 2 * math.pi + temp_target_direction
        else:
            return temp_target_direction # 0 ~ 2pi
        

def main(args=None):
    rclpy.init(args=args)

    flight_strategy = DirectFly("drone")

    rclpy.spin(flight_strategy)
    flight_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()