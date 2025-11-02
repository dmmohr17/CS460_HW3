import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
from nav_msgs.msg import Odometry
from collections import deque


class Walk(Node):
    def __init__(self):
        super().__init__('Walk')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/base_scan', self.sensor_callback, 20)
        # self.subscription_odom = self.create_subscription(Odometry, '/ground_truth', self.listener_callback, 20)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Set baseline numbers
        self.desired_wall_distance = 0.5
        self.safe_forward_speed = 0.2
        self.tick = 0

        # Laser scan data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.move_cmd = Twist()

    # Sensor callback
    def sensor_callback(self, msg: LaserScan):

        twist = Twist()

        def filter_vals(range):
            range = [v for v in range if not math.isinf(v) and not math.isnan(v)]
            return min(range) if range else float('inf')

        n = len(msg.ranges)
        front = n // 2
        right = n // 6
        left = n - right
        far_right = 1
    
        self.front_distance = filter_vals(msg.ranges[front-5:front+5])
        print("front: ", self.front_distance)
        self.right_distance = filter_vals(msg.ranges[right-5:right+5])
        print("right: ", self.right_distance)
        self.left_distance = filter_vals(msg.ranges[left-5:left+5])
        print("left: ", self.left_distance)
        self.far_right_distance = msg.ranges[far_right]
        print("far right: ", self.far_right_distance)
        print("")

        # Not front and not right - heading away from wall, need to turn right back toward wall
        if(self.front_distance >= 1 and self.right_distance >= 1):
            twist.linear.x = 0.1
            twist.angular.z = -0.5
            if(self.far_right_distance < 0.5):
                twist.angular.z = -1.0

        # Front and not right - away from wall but toward obstacle, need to turn left
        if(self.front_distance < 1 and self.right_distance >= 1):
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        # Not front and right - following wall, drive forward
        if(self.front_distance >= 1 and self.right_distance < 1):
            twist.linear.x = self.safe_forward_speed
            twist.angular.z = 0.0
            # Course correct to stay parallel
            # Drifting away, turn slightly right toward wall 
            if(msg.ranges[right-1] < msg.ranges[right+1]):
                twist.angular.z = -0.1
            # Drifting toward, turn slightly left away from wall
            if(msg.ranges[right-1] > msg.ranges[right+1]):
                twist.angular.z = 0.1
            # Anti-crash mechanism - don't hug the wall too close
            if(self.right_distance < 0.3):
                twist.angular.z = 0.2

        # Front and right - at corner, need to turn left
        if(self.front_distance < 1 and self.right_distance < 1):
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        # Unknown state
        

        
        self.cmd_pub.publish(twist)

    # Timer callback
    def timer_callback(self):
        self.tick += 1




def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Walk()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
