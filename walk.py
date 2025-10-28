import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
from collections import deque

# no ground truth - but you can use local estimate
# wall following is a hack
# step 1 - scan for a wall - look at architecture reactive control slide - define danger/safe zones
# need to define left, right, and front of robot
# [0 - 5] -> 0 = obstacle, 5 = no obstacle
# PID controller -> error = distance from wall (x(t)-g(t)) -> goal is distance from the wall (ex: stay 3m from wall)
# Can also include random, but easiest is to pick one side with wall following. If something is in front of you,
# ...always turn right (or whichever direction you chose).
# Biggest issue will be corners, will need to solve for that - check architecture slides
# Will need to figure out how to map PID to linear & angular

Laser scan info: angle_min=-2.36, angle_max=2.36
angle_increment=0.0175, total points=270

class Walk(Node):
    def __init__(self):
        super().__init__('Walk')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/base_scan', self.sensor_callback, 20)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Laser scan data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # may want to tune these values if wall crashing occurs
        self.danger_zone = 0.3
        self.follow_distance = 0.6
        self.wall_found = False
        self.start = True
        self.start_helper = 0.0
        self.temp = True

        self.move_cmd = Twist()

    # Sensor callback
    def sensor_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if self.temp:
            print(f"Laser scan info: angle_min={msg.angle_min:.2f}, "
                f"angle_max={msg.angle_max:.2f}")
            print(f"angle_increment={msg.angle_increment:.4f}, "
                f"total points={len(msg.ranges)}")

        # estimates closest obstacles in front, left, and right
        if n > 0:
            self.front_distance = min(msg.ranges[n//2 - 5 : n//2 + 5])
            self.left_distance = min(msg.ranges[-15:])
            self.right_distance = min(msg.ranges[:15])

    # Timer callback
    def timer_callback(self):
        twist = Twist()

        if(self.start == True):
            # at init, need to find a wall
            if(self.wall_found == False):
                if(self.front_distance == self.left_distance and self.front_distance == self.right_distance):
                    self.start_helper += 1.0
                    if(self.start_helper >= 20):
                        # taken too long to find a wall, drive until one is found
                        twist.angular.z = 0.2
                        twist.linear.x = 0.4
                    else:
                        # no walls found, keep rotating
                        twist.angular.z = 0.5
                        twist.linear.x = 0.0
                else:
                    # wall found, sit stil for now, will move to other block on next loop
                    self.wall_found = True
                    print("Wall found!")
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
            else:
                # wall found, going to it
                if(self.right_distance <= self.follow_distance):
                    # reached wall, now follow it, and mark start phase as over
                    self.start = False
                    print("Aligned with wall!")
                    twist.angular.z = 0.0
                    twist.linear.x = 0.2
                elif(self.front_distance <= self.follow_distance):
                    # rotate so right side is within follow distance
                    twist.angular.z = 0.2
                    twist.linear.x = 0.0
                elif(self.left_distance <= self.follow_distance):
                    # rotate left so front will eventually be in follow distance
                    twist.angular.z = 0.2
                    twist.linear.x = 0.0
                elif(self.left_distance < self.front_distance and self.left_distance < self.right_distance):
                    # if left side is closest wall, turn that way
                    twist.angular.z = 0.2
                    twist.linear.x = 0.0
                elif(self.front_distance <= self.left_distance and self.front_distance <= self.right_distance):
                    # go towards wall
                    twist.angular.z = 0.0
                    twist.linear.x = 0.2
                else:
                    # right is closest, turn right to eventually face it
                    twist.angular.z = -0.2
                    twist.linear.x = 0.0
        elif(self.wall_found == True and self.right_distance > self.follow_distance + 0.5):
            # if robot was following a wall and the wall disappears, turn right until found again - INCOMPLETE
            twist.angular.z = -0.1
            twist.linear.x = 0.1
        elif(self.wall_found == True and self.front_distance < self.follow_distance):
            # if robot is following a wall and a wall is in front, turn left slowly
            twist.angular.z = 0.2
            twist.linear.x = 0.0
        else:
            # follow the wall - but check for odd cases
            if(self.right_distance < self.danger_zone):
                if(self.left_distance < self.danger_zone and self.front_distance > self.danger_zone):
                    # if in a tight corridor but there is space ahead, just go straight
                    twist.angular.z = 0.0
                    twist.linear.x = 0.2
                else:
                    # turn left slightly
                    twist.angular.z = 0.1 # turn left
                    twist.linear.x = 0.2
            elif(self.right_distance > self.follow_distance): 
                # if the robot is not parallel with the wall, turn closer to it
                twist.angular.z = -0.1 # rotate right slightly to get parallel with wall
                twist.linear.x = 0.2
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.2

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Walk()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
