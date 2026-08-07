#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def bresenham_line(x0, y0, x1, y1):
    """Returns grid cell coordinates along a ray from (x0, y0) to (x1, y1)."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    curr_x, curr_y = x0, y0
    while True:
        points.append((curr_x, curr_y))
        if curr_x == x1 and curr_y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            curr_x += sx
        if e2 < dx:
            err += dx
            curr_y += sy
    return points


def yaw_from_quaternion(q):
    """Extracts yaw angle (in radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GridMap(Node):
    def __init__(self):
        super().__init__('grid_map')

        # 1. Publishers and Subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 2. TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Grid Map Parameters
        self.resolution = 0.05  # 5cm per cell
        self.width = 1000        # 200 cells (10 meters wide)
        self.height = 1000       # 200 cells (10 meters tall)
        self.origin_x = - (self.width * self.resolution) / 2.0   # -5.0m
        self.origin_y = - (self.height * self.resolution) / 2.0  # -5.0m

        # Log-odds parameters
        self.L_FREE = -0.4     # Decrease log-odds for clear cells
        self.L_OCC = 0.85      # Increase log-odds for hit cells
        self.L_MIN = -5.0      # Min clamping limit
        self.L_MAX = 5.0       # Max clamping limit

        # Map arrays
        self.grid_log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.observed = np.zeros((self.height, self.width), dtype=bool)

        # Publish map periodically (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info("Occupancy Grid Map node started!")

    def world_to_grid(self, x, y):
        """Converts world coordinates (meters) to grid indices (col, row)."""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return col, row

    def is_in_grid(self, col, row):
        """Checks if cell coordinates are within grid bounds."""
        return 0 <= col < self.width and 0 <= row < self.height

    def scan_callback(self, scan_msg: LaserScan):
        # Lookup transform from 'map' to the incoming laser scan frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                scan_msg.header.frame_id,
                rclpy.time.Time(),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform 'map' to '{scan_msg.header.frame_id}': {ex}")
            return

        # Extract sensor position and yaw in the map frame
        sensor_x = transform.transform.translation.x
        sensor_y = transform.transform.translation.y
        sensor_yaw = yaw_from_quaternion(transform.transform.rotation)

        sensor_col, sensor_row = self.world_to_grid(sensor_x, sensor_y)
        if not self.is_in_grid(sensor_col, sensor_row):
            return

        # Process each laser beam (downsample by 2 for speed if needed)
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if math.isnan(r) or r < scan_msg.range_min:
                angle += scan_msg.angle_increment
                continue

            # Cap max range measurements
            is_hit = True
            if r >= scan_msg.range_max or math.isinf(r):
                r = scan_msg.range_max
                is_hit = False

            # Beam endpoint in global world frame
            hit_x = sensor_x + r * math.cos(sensor_yaw + angle)
            hit_y = sensor_y + r * math.sin(sensor_yaw + angle)
            hit_col, hit_row = self.world_to_grid(hit_x, hit_y)

            # Raycast from sensor position to endpoint
            ray_cells = bresenham_line(sensor_col, sensor_row, hit_col, hit_row)

            # Update free cells along the ray
            for col, row in ray_cells[:-1]:
                if self.is_in_grid(col, row):
                    self.grid_log_odds[row, col] += self.L_FREE
                    self.observed[row, col] = True

            # Update occupied cell at endpoint (if ray hit a physical obstacle)
            if is_hit:
                end_col, end_row = ray_cells[-1]
                if self.is_in_grid(end_col, end_row):
                    self.grid_log_odds[end_row, end_col] += self.L_OCC
                    self.observed[end_row, end_col] = True

            angle += scan_msg.angle_increment

        # Clamp log-odds to prevent over-saturation
        np.clip(self.grid_log_odds, self.L_MIN, self.L_MAX, out=self.grid_log_odds)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0

        # Standard Sigmoid function: p = 1 / (1 + e^-l)
        prob = 1.0 / (1.0 + np.exp(-self.grid_log_odds))
        occupancy_data = (prob * 100).astype(np.int8)

        # Mark unobserved space as -1 (Unknown) for RViz
        occupancy_data[~self.observed] = -1

        # Flatten into 1D row-major array
        msg.data = occupancy_data.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()