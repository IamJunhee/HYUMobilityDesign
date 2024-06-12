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
        range_arr = np_replace_inf_to(np.array(self.drone_data.lidar.ranges), self.drone_data.lidar.range_max)
        range_arr = np.exp(range_arr)
        obstacle_prob_arr = range_arr/np.sum(range_arr)

        direction_arr = np.array((
                -(self.drone_data.target.longitude - self.drone_data.gps.longitude),
                -(self.drone_data.target.latitude - self.drone_data.gps.latitude)
            ))

        direction_arr = direction_arr / np.linalg.norm(direction_arr, 2)

        temp_target_direction = math.atan2(direction_arr[1], direction_arr[0])
        target_direction = (temp_target_direction + 2 * math.pi) \
                                    if temp_target_direction < 0 \
                                    else temp_target_direction

        angle_array = np.arange(self.drone_data.lidar.angle_min,self.drone_data.lidar.angle_max+self.drone_data.lidar.angle_increment,self.drone_data.lidar.angle_increment)

        angle_array2 = target_direction - angle_array
        angle_array2 = (np.cos(angle_array2)+1)/ 2 / 32.9978919434732

        obs_prob_arr2 = obstacle_prob_arr * angle_array2
        print(np.sum(obstacle_prob_arr))
        print(np.sum(angle_array2))
        print(np.sum(obs_prob_arr2))
        raise Exception()
        expected_val = np.sum(obs_prob_arr2 * angle_array)

        self.get_logger().info("Target Direction : %f" % target_direction)
        self.get_logger().info("expected Direction : %f" % expected_val)

        return 15, expected_val
        
    def calculate_z_velocity(self):
        err = self.drone_data.target.altitude - self.drone_data.gps.altitude + 10
        return err


def np_replace_inf_to(arr, val):
    range_arr = np.copy(arr)
    np.putmask(range_arr, np.isinf(arr), val)

    return range_arr

def main(args=None):
    rclpy.init(args=args)

    flight_strategy = ObstacleAvoidanceFly("drone")

    rclpy.spin(flight_strategy)
    flight_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()