import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
from collections import deque

# wall following is a hack
# step 1 - scan for a wall - look at architecture reactive control slide - define danger/safe zones
# need to define left, right, and front of robot
# [0 - 5] -> 0 = obstacle, 5 = no obstacle
# PID controller -> error = distance from wall (x(t)-g(t)) -> goal is distance from the wall (ex: stay 3m from wall)
# Can also include random, but easiest is to pick one side with wall following. If something is in front of you,
# ...always turn right (or whichever direction you chose).
# Biggest issue will be corners, will need to solve for that - check architecture slides
# Will need to figure out how to map PID to linear & angular

# Derek's notes
# Turning right at a corner has issues when the wall is extremely small (like a line)
# Issues with obstacles slightly to left or right (increase size of front, left, and right sensors?)
# When robot senses a very small gap, it tries to turn (doesn't have logic for if a gap is too small to fit)
# Logic could use tidying up / refinement
# Need to add PID - may help with tuning
# May want to tune values for follow distance and danger zone
# Add diagonal sensors?

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
        self.danger_zone = 0.4
        self.follow_distance = 0.6
        self.wall_found = False
        self.start = True
        self.start_helper = 0.0 # helps check for if robot is turning in a circle for too long at init

        # for print statements
        self.tick = 0
        self.ticks = "temp"

        # memory
        self.turningRight = False
        self.turningLeft = False

        # linear PID - will want to tune these values
        self.lin_KP = 1.0
        self.lin_KI = 0.0
        self.lin_KD = 0.1
        self.lin_e = 1
        self.lin_esum = 1
        self.lin_eprev = 1
        self.lin_dedt = 1
        self.lin_dt = 1
        self.lin_PID = 1
        self.lin_u = 1

        # angular pid - will want to tune these values
        self.ang_KP = 1
        self.ang_KI = 0.0
        self.ang_KD = 0.2
        self.ang_e = 1
        self.ang_esum = 1
        self.ang_eprev = 1
        self.ang_dedt = 1
        self.ang_dt = 1
        self.ang_PID = 1
        self.lin_u = 1

        self.move_cmd = Twist()

    # Sensor callback
    def sensor_callback(self, msg: LaserScan):

        # filter sensor values for invalid readings
        def filter_vals(range):
            range = [v for v in range if not math.isinf(v) and not math.isnan(v)]
            return min(range) if range else float('inf')
        
        n = len(msg.ranges)
        front = n // 2
        right = n // 6
        left = n - right

        # estimates closest obstacles in front, left, and right
        if n > 0:
            self.front_distance = filter_vals(msg.ranges[front - 5 : front + 5])
            self.left_distance  = filter_vals(msg.ranges[left - 5  : left + 5])
            self.right_distance = filter_vals(msg.ranges[right - 5 : right + 5])

        # PID code - imcomplete, and needs to be integrated into timer_callback
        # also, not sure if the below calculations are correct (they were hastily made) - feel free to fix them
        lin_e = self.front_distance - self.follow_distance
        ang_e = self.right_distance - self.follow_distance

        self.lin_esum = self.lin_esum + (lin_e * self.lin_dt)
        self.lin_dedt = (lin_e - self.lin_eprev) / self.lin_dt

        self.ang_esum = self.ang_esum + (lin_e * self.ang_dt)
        self.ang_dedt = (ang_e - self.ang_eprev) / self.ang_dt

        self.lin_u = self.lin_KP * lin_e + self.lin_KI * self.lin_esum + self.lin_KD * self.lin_dedt
        self.ang_u = self.ang_KP * ang_e + self.ang_KI * self.ang_esum + self.ang_KD * self.ang_dedt

        # probably need to add more calculations - Solution incomplete

        self.lin_eprev = lin_e
        self.ang_eprev = ang_e

    # Timer callback
    def timer_callback(self):
        twist = Twist()
        self.tick += 1
        self.ticks = str(self.tick)

        if(self.front_distance > self.danger_zone + 0.1):
            self.turningLeft = False
            self.turningRight = False

        if(self.start == True):
            # at init, need to find a wall
            if(self.wall_found == False):
                if(abs(self.front_distance - self.left_distance) < 0.05 and abs(self.front_distance - self.right_distance) < 0.05):
                    self.start_helper += 1
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
                    twist.linear.x = 0.0
                elif(self.front_distance <= self.follow_distance and self.turningRight == False):
                    # rotate so right side is within follow distance
                    self.turningLeft = True
                    twist.angular.z = 0.2
                    twist.linear.x = 0.0
                elif(self.left_distance <= self.follow_distance and self.turningRight == False):
                    # rotate left so front will eventually be in follow distance
                    self.turningLeft = True
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
                    self.turningRight = True
                    twist.angular.z = -0.2
                    twist.linear.x = 0.0
        # cases after initial wall finding
        elif(self.front_distance <= self.danger_zone):
            # something is dangerously close to front
            if(self.right_distance <= self.follow_distance and self.left_distance > self.follow_distance and self.turningRight == False):
                # if robot is following a wall and a wall is in front, turn left slowly
                print("Wall in front and on right, turn left" + self.ticks)
                twist.angular.z = 0.2
                twist.linear.x = 0.0
                self.turningLeft = True
            elif(self.turningLeft == False):
                print("Wall in front, going to turn right")
                twist.angular.z = -0.2
                twist.linear.x = 0.0
                self.turningRight = True
            else:
                print("self.TurningLeft is still true")
                twist.angular.z = 0.2
                twist.linear.x = 0.0
        elif(self.left_distance <= self.danger_zone and self.right_distance > self.danger_zone):
            print("Danger on left triggered" + self.ticks)
            twist.angular.z = -0.2
            if(self.front_distance > self.follow_distance):
                twist.linear.x = 0.1
            else:
                twist.linear.x = 0.0
        else: # NEW PID ELSE BLOCK - REPLACED BELOW CODE
            # testing PID solution
            print("PID section")
            twist.angular.z = self.ang_u
            twist.linear.x = self.lin_x
        # OLD NON PID CODE BELOW - UNCOMMENT IF THIS DOES NOT WORK
        """
        elif(self.right_distance > self.follow_distance + 0.2):
            # if robot was following a wall and the wall disappears, turn right until found again
            print("Wall lost, turning right and searching" + self.ticks)
            twist.angular.z = -0.5
            twist.linear.x = 0.1
        else:
            # follow the wall - but check for odd cases
            if(self.right_distance <= self.danger_zone):
                if(self.left_distance <= self.danger_zone and self.front_distance > self.follow_distance):
                    # if in a tight corridor but there is space ahead, just go straight
                    print("Tight on both sides, going straight" + self.ticks)
                    twist.angular.z = 0.0
                    twist.linear.x = 0.2
                else:
                    # turn left slightly
                    print("Right in danger zone, block ahead" + self.ticks)
                    twist.angular.z = 0.2 # turn left
                    twist.linear.x = 0.0
                    self.turningLeft = True
            elif(self.right_distance > self.follow_distance): 
                # if the robot is not parallel with the wall, turn closer to it
                print("Robot misaligned with wall, turn right" + self.ticks)
                twist.angular.z = -0.2 # rotate right slightly to get parallel with wall
                twist.linear.x = 0.1
            else:
                print("Perfect alignment, going straight" + self.ticks)
                twist.angular.z = 0.0
                twist.linear.x = 0.2
        """

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Walk()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
