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
        self.subscription_odom = self.create_subscription(Odometry, '/ground_truth', self.listener_callback, 20)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Robot position and heading
        self.x = 0.0
        self.y = 0.0
        self.degrees = 0.0

        # Start setup
        self.isStart = True
        self.startPos = [0.0, 0.0]
        self.startTime = time.time()

        self.map_resolution = 0.25        # resolution per grid cell
        self.map_size = 16.0              # map is 16x16
        self.grid_dim = int(self.map_size / self.map_resolution) # internal grid 
        self.arr = [[0 for _ in range(self.grid_dim)] for _ in range(self.grid_dim)]
        # 0=unknown, 1=wall, 2=free
        self.path = []
        self.last_plan_time = 0.0
        self.goal_x = None
        self.goal_y = None
        self.safe_clearance = 0.1

        self.visited = [[False] * len(self.arr) for _ in range(len(self.arr))]
        self.last_goal_update = time.time()
        self.current_target = None

        # Laser scan data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.move_cmd = Twist()

    # Visualization
    def display_map_ascii(self):
    # Prints current visual of map constantly
        display = ""
        robot_gx, robot_gy = self.world_to_grid(self.x, self.y)
        goal_gx, goal_gy = self.world_to_grid(self.goal_x, self.goal_y) if self.goal_x is not None else (-1, -1)

        for y in reversed(range(self.grid_dim)):
            line = ""
            for x in range(self.grid_dim):
                if x == robot_gx and y == robot_gy:
                    line += "R" # Robot
                elif x == goal_gx and y == goal_gy:
                    line += "G" # Goal
                elif self.arr[y][x] == 0:
                    line += "░" # Unknown
                elif self.arr[y][x] == 1:
                    line += "W" # Wall
                elif self.arr[y][x] == 2:
                    line += "·" # Free
                else:
                    line += "?" # Error
            display += line + "\n"
        # print(display)

    # Sensor callback
    def sensor_callback(self, msg: LaserScan):
        # Update occupancy grid from sensor data
        self.whisker_array = msg.ranges
        angle_inc = msg.angle_increment
        angle_min = msg.angle_min
        max_range = msg.range_max if msg.range_max > 0 else 5.0  # fallback if unspecified

        robot_gx, robot_gy = self.world_to_grid(self.x, self.y)

        # Mark a small free circle around the robot
        # This checks safety since the robot is larger than a dot
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                gx = robot_gx + dx
                gy = robot_gy + dy
                if 0 <= gx < self.grid_dim and 0 <= gy < self.grid_dim:
                    self.arr[gy][gx] = 2

        # Process each ray
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or r <= 0.01:
                continue

            # Cap extremely large values but store max range
            r = min(r, max_range)
            global_angle = math.radians(self.degrees) + (angle_min + i * angle_inc)
    
            # Compute world coordinates of ray endpoint
            wx = self.x + r * math.cos(global_angle)
            wy = self.y + r * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)

            # Trace along the ray and mark free space
            steps = int(r / self.map_resolution)
            for s in range(steps):
                fx = self.x + s * self.map_resolution * math.cos(global_angle)
                fy = self.y + s * self.map_resolution * math.sin(global_angle)
                fx_g, fy_g = self.world_to_grid(fx, fy)
                if 0 <= fx_g < self.grid_dim and 0 <= fy_g < self.grid_dim:
                    if self.arr[fy_g][fx_g] == 0:
                        self.arr[fy_g][fx_g] = 2
    
            # Only mark a wall if the ray was shorter than max range
            if r < max_range * 0.99:
                if 0 <= gx < self.grid_dim and 0 <= gy < self.grid_dim:
                    self.arr[gy][gx] = 1
    

    # Odometry listener
    def listener_callback(self, msg):
        # Calculate angles from quaternion provided
        w = msg.pose.pose.orientation.w
        z = msg.pose.pose.orientation.z
        theta = (2.0 * math.atan2(z, w))
        degrees = math.degrees(theta)
        if degrees < 0: # Loop over for negative degrees
            degrees += 360
        self.degrees = degrees

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x = x
        self.y = y

        if self.isStart: # Set goal based on initial robot location
            if x < 0:
                self.goal_x = 8
            else:
                self.goal_x = -8
            if y < 0:
                self.goal_y = 8
            else:
                self.goal_y = -8
            self.isStart = False
            self.startPos = [x, y]

        dx = x - self.startPos[0]
        dy = y - self.startPos[1]
        distanceTraveled = math.sqrt(dx * dx + dy * dy)

        curTime = time.time()
        elapsed = curTime - self.startTime

        # print("Elapsed Time:", elapsed)
        # print("Distance:", distanceTraveled)

        cell_x, cell_y = self.world_to_grid(self.x, self.y)
        self.visited[cell_y][cell_x] = True


    # Coordinate Conversion
    def world_to_grid(self, x, y):
        gx = int((x + 8) / self.map_resolution)
        gy = int((y + 8) / self.map_resolution)
        gx = max(0, min(self.grid_dim - 1, gx))
        gy = max(0, min(self.grid_dim - 1, gy))
        return gx, gy


    # Timer callback
    def timer_callback(self):
        # print(self.display_map_ascii()) # Display current map

        if self.goal_x is None or self.goal_y is None:
            sx, sy = self.x, self.y
            self.goal_x = self.map_size - sx
            self.goal_y = self.map_size - sy
            # self.get_logger().info(f"Goal set to ({self.goal_x:.2f}, {self.goal_y:.2f})")

        start = self.world_to_grid(self.x, self.y)
        goal = self.world_to_grid(self.goal_x, self.goal_y)

        # Track the current stats of the map
        free_count = sum(row.count(2) for row in self.arr)
        wall_count = sum(row.count(1) for row in self.arr)
        # self.get_logger().info(f"Map coverage: free={free_count}, walls={wall_count}")

        # Replan occasionally
        if (not self.path) or (time.time() - self.last_plan_time > 2.0):
            self.path = self.compute_a_star(start, goal)
            self.last_plan_time = time.time()
            if not self.path:
                # self.get_logger().info("No path found yet.")
                return

        if not self.path:
            return

        next_cell = self.path[0]
        next_world_x = next_cell[0] * self.map_resolution - 8
        next_world_y = next_cell[1] * self.map_resolution - 8

        dx = next_world_x - self.x
        dy = next_world_y - self.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        angle_to_target = math.degrees(math.atan2(dy, dx))
        heading_error = (angle_to_target - self.degrees + 540) % 360 - 180

        twist = Twist()
        if abs(heading_error) > 10:
            twist.angular.z = 3.2 * (heading_error / abs(heading_error))
            twist.linear.x = 0.0
        else:
            twist.angular.z = 1.0 * heading_error / 10.0
            twist.linear.x = 1.6

        if dist < self.map_resolution * 0.8:
            self.path.pop(0)

        self.cmd_pub.publish(twist)


    # A* Search
    def compute_a_star(self, start, goal):
        from heapq import heappush, heappop
        grid = self.arr
        dim = self.grid_dim
        clearance = max(1, int(self.safe_clearance / self.map_resolution) - 1)

        def in_bounds(n):
            return 0 <= n[0] < dim and 0 <= n[1] < dim

        def passable(n):
            x, y = n
            for dx in range(-clearance, clearance + 1):
                for dy in range(-clearance, clearance + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < dim and 0 <= ny < dim:
                        if grid[ny][nx] == 1:
                            return False
            return grid[y][x] != 1

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        frontier = []
        heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heappop(frontier)
            if current == goal:
                break
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nxt = (current[0] + dx, current[1] + dy)
                if not in_bounds(nxt) or not passable(nxt):
                    continue
                new_cost = cost_so_far[current] + 1
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + heuristic(goal, nxt)
                    heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        if goal not in came_from:
            return []

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Walk()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

