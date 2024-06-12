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
        obstacle_prob_arr = np.log(range_arr / 2)

        direction_arr = np.array((
                -(self.drone_data.target.longitude - self.drone_data.gps.longitude),
                -(self.drone_data.target.latitude - self.drone_data.gps.latitude)
            ))

        direction_arr = direction_arr / np.linalg.norm(direction_arr, 2)

        temp_target_direction = math.atan2(direction_arr[1], direction_arr[0])
        target_direction = (temp_target_direction + 2 * math.pi) \
                                    if temp_target_direction < 0 \
                                    else temp_target_direction

        angle_array = np.linspace(self.drone_data.lidar.angle_min, self.drone_data.lidar.angle_max, len(self.drone_data.lidar.ranges))
        angle_array += self.drone_data.heading_direction
        np.putmask(angle_array, angle_array < 0, angle_array + 2 * np.pi)
        angle_array2 = angle_array - target_direction 
        angle_array2 = np.cos(angle_array2)

        obs_prob_arr2 = obstacle_prob_arr * angle_array2
        obs_prob_arr2 = sum_around(obs_prob_arr2, 100)
        max_index = np.argmax(obs_prob_arr2)
        expected_val = angle_array[max_index]

        self.get_logger().info("Target Direction : %f" % target_direction)
        self.get_logger().info("expected Direction : %f" % expected_val)

        return 30, expected_val
        
    def calculate_z_velocity(self):
        err = self.drone_data.target.altitude - self.drone_data.gps.altitude + 10
        return err

def sum_around(arr, _range):
    temp_arr = np.copy(arr)
    last_index = len(temp_arr) - 1
    for i in range(len(temp_arr)):
        sum_index = np.array(range(i - _range, i + _range + 1))
        np.putmask(sum_index, sum_index > last_index, sum_index - last_index - 1)
        temp_arr[i] = np.sum(temp_arr[sum_index]) / (2 * _range + 1)

    return temp_arr

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