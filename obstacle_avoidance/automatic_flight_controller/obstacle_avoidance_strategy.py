from automatic_flight_controller.fly_strategy import FlyStrategy
import numpy as np
import math
import rclpy

# 충분한 높이까지 상승하여 직진으로 비행하는 방법
class ObstacleAvoidanceFly(FlyStrategy):
    def __init__(self, drone_name):
        super().__init__(drone_name)
        self.get_logger().info("Obstacle Avoidance Strategy is started")

    def calculate_xy_velocity(self):
        pass
        
    def calculate_z_velocity(self):
        pass

        

def main(args=None):
    rclpy.init(args=args)

    flight_strategy = ObstacleAvoidanceFly("drone")

    rclpy.spin(flight_strategy)
    flight_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()