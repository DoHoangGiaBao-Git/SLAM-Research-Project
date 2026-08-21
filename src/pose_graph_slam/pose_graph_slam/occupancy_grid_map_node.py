#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        # === Map parameters ===
        self.resolution = 0.05          # meters / cell
        self.width_m = 40.0             # map extent in meters
        self.height_m = 40.0
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # Log-odds parameters
        self.l0 = 0.0
        self.l_occ = 1.0     
        self.l_free = -0.15  
        self.l_min, self.l_max = -6.0, 6.0  

        self.log_odds = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)

        # Storage for keyframe scans: [(x_base, y_base, is_hit), ...]
        # Populated 1:1, in order, from /keyframe_scan - no guessing which
        # scan belongs to which pose-graph node.
        self.keyframes = []

        # === Topics ===
        self.create_subscription(LaserScan, '/keyframe_scan', self.keyframe_scan_callback, 50)
        self.create_subscription(PoseArray, '/pose_graph_nodes', self.pose_graph_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        self.get_logger().info('Occupancy Grid Node active with Keyframe Re-projection.')

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width_cells and 0 <= gy < self.height_cells

    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while (x, y) != (x1, y1):
            cells.append((x, y))
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells

    # Convert an incoming keyframe scan to local robot base frame points and
    # store it directly - each message here corresponds to exactly one
    # pose-graph node, in the same order the node was created
    def keyframe_scan_callback(self, msg: LaserScan):
        points = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            is_hit = msg.range_min <= r <= msg.range_max
            r_used = r if is_hit else msg.range_max

            x_lidar = r_used * math.cos(angle)
            y_lidar = r_used * math.sin(angle)
            x_base = -y_lidar - 0.040
            y_base = x_lidar

            points.append((x_base, y_base, is_hit))

        self.keyframes.append(points)

    def pose_graph_callback(self, msg: PoseArray):
        num_poses = len(msg.poses)

        # Keyframes are appended 1:1 by keyframe_scan_callback as nodes are
        # created. If a keyframe hasn't arrived yet, wait for the next
        # PoseArray update rather than guessing its scan.
        if len(self.keyframes) < num_poses:
            return

        # Reset log-odds matrix to unbake old positions
        self.log_odds.fill(0.0)

        # Re-project all keyframe scans using updated pose locations
        for idx, pose_msg in enumerate(msg.poses):
            if idx >= len(self.keyframes):
                break

            px = pose_msg.position.x
            py = pose_msg.position.y
            qz = pose_msg.orientation.z
            qw = pose_msg.orientation.w
            ptheta = 2.0 * math.atan2(qz, qw)

            sgx, sgy = self.world_to_grid(px, py)
            cos_t = math.cos(ptheta)
            sin_t = math.sin(ptheta)

            for x_base, y_base, is_hit in self.keyframes[idx]:
                ex = px + cos_t * x_base - sin_t * y_base
                ey = py + sin_t * x_base + cos_t * y_base
                egx, egy = self.world_to_grid(ex, ey)

                # Free space raycasting
                for gx, gy in self.bresenham(sgx, sgy, egx, egy):
                    if self.in_bounds(gx, gy):
                        self.log_odds[gy, gx] = np.clip(
                            self.log_odds[gy, gx] + self.l_free - self.l0, self.l_min, self.l_max
                        )

                # Occupied endpoint
                if is_hit and self.in_bounds(egx, egy):
                    self.log_odds[egy, egx] = np.clip(
                        self.log_odds[egy, egx] + self.l_occ - self.l0, self.l_min, self.l_max
                    )

        self.publish_map()

    def publish_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width_cells
        grid_msg.info.height = self.height_cells
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.orientation.w = 1.0

        prob = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        data = np.full(self.log_odds.shape, -1, dtype=np.int8)
        known = self.log_odds != 0.0
        data[known] = (prob[known] * 100).astype(np.int8)

        grid_msg.data = data.flatten().tolist()
        self.map_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()