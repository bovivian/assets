import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from graphslam_msgs.srv import GetPose
import matplotlib.pyplot as plt
import math
import time

class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('turtlebot_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.gazebo_subscriber = self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)
        self.cmd = Twist()
        self.position = None
        self.orientation = None
        self.yaw = None
        self.initial_yaw = None
        self.gazebo_position = None
        self.slam_position = None
        self.positions = []
        self.errors = []

        # Wait for initial odometry data
        while self.yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.initial_yaw = self.yaw

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.yaw = self.euler_from_quaternion(self.orientation)

    def gazebo_callback(self, msg):
        try:
            index = msg.name.index('turtlebot3_waffle')
            pose = msg.pose[index]
            self.gazebo_position = (pose.position.x, pose.position.y)
        except ValueError:
            self.get_logger().warn('Turtlebot model not found in gazebo model states')

    def euler_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def move_by_distance(self, linear_speed, target_distance):
        time.sleep(0.5)
        initial_position = self.get_position()
        if initial_position is None:
            return

        initial_x, initial_y = initial_position[0], initial_position[1]
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd.linear.z = 0.0
        
        total_distance = 0.0
        current_speed = 0.0
        acceleration = 0.005
        deceleration_distance = 0.5 
        
        min_speed = 0.08

        while total_distance < target_distance:
            if current_speed < linear_speed and total_distance < (target_distance - deceleration_distance):
                current_speed += acceleration
                if current_speed > linear_speed:
                    current_speed = linear_speed

            if total_distance >= (target_distance - deceleration_distance):
                current_speed -= acceleration
                if current_speed < min_speed:
                    current_speed = min_speed

            self.cmd.linear.x = current_speed
            self.publisher.publish(self.cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

            current_position = self.get_position()
            if current_position is None:
                continue

            current_x, current_y = current_position[0], current_position[1]
            total_distance = math.sqrt((current_x - initial_x) ** 2 + (current_y - initial_y) ** 2)

        self.stop()


    def turn_to_angle(self, target_angle):
        time.sleep(0.5)
        target_angle_rad = math.radians(target_angle)
        
        target_yaw = self.normalize_angle(self.initial_yaw + target_angle_rad)
        
        self.cmd.linear.x = 0.0
        Kp = 0.5 
        Ki = 0.1 
        Kd = 0.2
        integral = 0.0
        previous_error = 0.0
        previous_time = self.get_clock().now().nanoseconds / 1e9
        
        while not self.is_angle_reached(target_yaw):
            current_time = self.get_clock().now().nanoseconds / 1e9
            delta_time = current_time - previous_time
            previous_time = current_time

            angle_diff = self.normalize_angle(target_yaw - self.yaw)
            integral += angle_diff * delta_time
            derivative = (angle_diff - previous_error) / delta_time if delta_time > 0 else 0.0
            previous_error = angle_diff

            control_signal = Kp * angle_diff + Ki * integral + Kd * derivative
            
            if abs(angle_diff) < math.radians(10):
                control_signal *= 0.7
            if abs(angle_diff) < math.radians(5):
                control_signal *= 0.5
            if abs(angle_diff) < math.radians(1):
                control_signal *= 0.4
            if abs(angle_diff) < math.radians(0.5):
                control_signal *= 0.3
            
            self.cmd.angular.z = control_signal
            self.publisher.publish(self.cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()

    def is_angle_reached(self, target_yaw):
        if self.yaw is None:
            return False
        angle_diff = abs(self.normalize_angle(target_yaw - self.yaw))
        return angle_diff < math.radians(0.2)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)

    def get_position(self):
        if self.position is not None:
            return (self.position.x, self.position.y, self.position.z)
        else:
            return None

    def get_orientation(self):
        if self.orientation is not None:
            return (self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)
        else:
            return None

    def get_slam_position(self):
        client = self.create_client(GetPose, 'get_pose')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        request = GetPose.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response:
            self.slam_position = (response.pose.position.x, response.pose.position.y)
        else:
            self.get_logger().warn('Failed to get SLAM position')

    def print_pos_and_orient(self):
        self.print_pos()
        self.print_orient()

    def print_gazebo_and_slam_positions(self):
        self.get_slam_position()
        if self.gazebo_position is None:
            while self.gazebo_position is None:
                rclpy.spin_once(self, timeout_sec=0.1)
        if self.gazebo_position and self.slam_position:
            x_g, y_g = self.gazebo_position
            x_s, y_s = self.slam_position
            self.get_logger().info(f'x_s: {x_s}, y_s: {y_s}, x_g: {x_g}, y_g: {y_g}')
            self.positions.append({
                "x_s": x_s,
                "y_s": y_s,
                "x_g": x_g,
                "y_g": y_g
            })
        else:
            self.get_logger().warn('Gazebo or SLAM position not available yet')

    def calculate_and_save_errors(self, filename, num):
        cumulative_squared_errors = []
        total_squared_error = 0.0
        mse_values = []
        rmse_values = []
        
        for point in self.positions:
            x_s, y_s = point["x_s"], point["y_s"]
            x_g, y_g = point["x_g"], point["y_g"]
            
            absolute_error = math.sqrt((x_s - x_g) ** 2 + (y_s - y_g) ** 2)
            squared_error = absolute_error ** 2
            total_squared_error += squared_error
            cumulative_squared_errors.append(total_squared_error)
            
            mse = total_squared_error / (len(cumulative_squared_errors))
            rmse = math.sqrt(mse)
            mse_values.append(mse)
            rmse_values.append(rmse)

        with open(filename + '_' + str(num) + '.txt', 'w') as f:
            for i, point in enumerate(self.positions):
                f.write(f'n {i+1}: slam = ({point["x_s"]}, {point["y_s"]}), gazebo = ({point["x_g"]}, {point["y_g"]})\n')
            f.write('\nMSE at each step: [' + ', '.join(f'{mse:.7f}' for mse in mse_values) + ']\n')
            f.write('RMSE at each step: [' + ', '.join(f'{rmse:.7f}' for rmse in rmse_values) + ']\n')
            f.write(f'\nFinal MSE: {mse_values[-1]:.7f}\n')
            f.write(f'Final RMSE: {rmse_values[-1]:.7f}\n')

        # Plotting MSE and RMSE over time
        plt.figure(figsize=(10, 5))
        plt.plot(mse_values, label='MSE')
        plt.plot(rmse_values, label='RMSE', linestyle='--')
        plt.xlabel('Measurement Step')
        plt.ylabel('Error Value')
        plt.title('MSE and RMSE over Time between SLAM and Gazebo Positions')
        plt.legend()
        plt.grid(True)
        plt.savefig(filename + '_plot_' + str(num) + '.png')
        plt.close()