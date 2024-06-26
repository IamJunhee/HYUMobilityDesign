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
        range_arr = np_replace_inf_to(np.array(self.drone_data.lidar.ranges), self.drone_data.lidar.range_max * 1.5)
        no_obstacle_prob_arr = map_function_to_array(self.__function_for_obstacle, range_arr)
        no_obstacle_prob_arr = no_obstacle_prob_arr / np.max(no_obstacle_prob_arr)

        target_direction = self.__calculate_target_angle()
        angle_array = self.__generate_angle_array()
        converted_angle_array = self.__convert_coordinate_to_world(angle_array)

        angle_diff_array = converted_angle_array - target_direction
        target_prob_arr = map_function_to_array(self.__function_for_target, angle_diff_array)

        prob_product_arr = no_obstacle_prob_arr * target_prob_arr
        prob_product_arr = sum_around(prob_product_arr, 150)
        
        max_index = np.argmax(prob_product_arr)
        expected_val = 0.9 * angle_array[max_index] + self.drone_data.heading_direction

        self.get_logger().info("Target Direction : %f" % target_direction)
        self.get_logger().info("expected Direction : %f" % expected_val)

        return 30, expected_val
    
    def __calculate_target_angle(self):
        direction_arr = np.array((
                -(self.drone_data.target.longitude - self.drone_data.gps.longitude),
                -(self.drone_data.target.latitude - self.drone_data.gps.latitude)
            ))

        direction_arr = direction_arr / np.linalg.norm(direction_arr, 2)

        target_direction = math.atan2(direction_arr[1], direction_arr[0])
        return (target_direction + 2 * math.pi) \
                                    if target_direction < 0 \
                                    else target_direction
    
    def __generate_angle_array(self):
        angle_array = np.linspace(self.drone_data.lidar.angle_min, self.drone_data.lidar.angle_max, len(self.drone_data.lidar.ranges))
        return angle_array
    
    def __convert_coordinate_to_world(self, arr):
        angle_array = np.copy(arr)
        angle_array += self.drone_data.heading_direction
        np.putmask(angle_array, angle_array < 0, angle_array + 2 * np.pi)

        return angle_array

    def __function_for_obstacle(self, x):
        val = (math.exp(x-5) - 1)
        val = val if val > 0 else 0
        val = val if val < math.exp(10) - 1 else math.exp(10) - 1
        return 100 * val

    def __function_for_target(self, x):
        return math.cos(x)
    
    def calculate_z_velocity(self):
        err = (self.drone_data.target.altitude - self.drone_data.gps.altitude) * 0.2
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

def map_function_to_array(func, arr):
    vect = np.vectorize(func)
    return vect(arr)

def main(args=None):
    rclpy.init(args=args)

    flight_strategy = ObstacleAvoidanceFly("drone")

    rclpy.spin(flight_strategy)
    flight_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()