import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
import random


class Walk(Node):
    def __init__(self):
        super().__init__('Walk')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/base_scan', self.sensor_callback, 20)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Set baseline numbers
        self.doorway_timer = 0
        self.turn_timer = 0
        self.tick = 0
        self.previous_right = float('inf')
        self.previous_left = float('inf')
        self.state = "following-wall"
        self.first_door = True

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
        first_door = True
    
        self.front_distance = filter_vals(msg.ranges[front-7:front+7])
        # print("front: ", self.front_distance)
        self.right_distance = filter_vals(msg.ranges[right-7:right+7])
        # print("right: ", self.right_distance)
        self.left_distance = filter_vals(msg.ranges[left-5:left+5])
        # print("left: ", self.left_distance)

        # initialize prev_right and prev_left
        if(self.previous_right == float('inf')):
            self.previous_right = self.right_distance
        if(self.previous_left == float('inf')):
            self.previous_left = self.left_distance

        match self.state:
            # base state is following a wall on right-hand side
            case "following-wall":
                print("following-wall")
                twist.linear.x = 0.5
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

                # if the front value gets too low, we are in a corner and need to turn left
                if(self.front_distance < 0.8):
                    self.state = "left-turn"

                # coin flip to decide if we explore doorway or not
                if(self.right_distance - self.previous_right > 1.0):
                    print("coin flip")
                    if(self.first_door == True):
                        self.first_door = False
                        self.state = "checking-right-doorway"
                    elif(random.randint(1,2) == 1):
                        self.state = "following-wall"
                    else:
                        self.state = "checking-right-doorway"


            # turn right for a little bit, then inch forward into doorway
            case "right-turn":
                print("right-turn")
                twist.linear.x = 0.0
                twist.angular.z = -1.0
                self.turn_timer += 1

                # if we are aligned with a wall mid-turn, stop turning 
                if(abs(msg.ranges[right-1] - msg.ranges[right+1]) < 0.005 and self.turn_timer > 5):
                    print("right-1", msg.ranges[right-1])
                    print("right+1", msg.ranges[right+1])
                    self.turn_timer = 0
                    self.state = "following-wall"

                # done turning 90 degrees, so inch forward
                if(self.turn_timer > 19 and self.turn_timer < 24):
                    # check for wall?
                    twist.linear.x = 0.15
                    twist.angular.z = 0.0

                if(self.turn_timer >= 24):
                    self.turn_timer = 0
                    self.state = "following-wall"

            # turn left until the area in front is clear
            case "left-turn":
                print("left-turn")
                twist.linear.x = 0.0
                twist.angular.z = 1.0

                if(self.front_distance > 1.0):
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0
                    self.state = "following-wall"

            case "checking-right-doorway":
                print("checking-right-doorway")
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.doorway_timer += 1

                # if timer hits a threshhold without sensing a new wall on the right, we've found a valid doorway
                if(self.doorway_timer > 8):
                    self.doorway_timer = 0
                    self.state = "right-turn"

                # if right sensor suddenly drops before hitting threshhold, doorway is too narrow
                if(self.right_distance - self.previous_right < -0.5 and self.right_distance < 1.0):
                    self.doorway_timer = 0
                    self.state = "following-wall"


        self.previous_right = self.right_distance
        self.previous_left = self.left_distance
        
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
