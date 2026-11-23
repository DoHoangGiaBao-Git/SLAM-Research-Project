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

        # Log-odds parameters (FIX: Higher l_occ and lower l_free prevent map fading)
        self.l0 = 0.0
        self.l_occ = 2.0       # Stronger weight for obstacle hits
        self.l_free = -0.05    # Gentle weight for free space
        self.l_min, self.l_max = -4.0, 8.0  

        self.log_odds = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)

        # Storage for keyframe scans: [(x_base, y_base, is_hit), ...]
        self.keyframes = []

        # === Topics ===
        self.create_subscription(LaserScan, '/keyframe_scan', self.keyframe_scan_callback, 50)
        self.create_subscription(PoseArray, '/pose_graph_nodes', self.pose_graph_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Periodic publish timer
        self.create_timer(0.5, self.publish_map)

        self.get_logger().info('Occupancy Grid Node active with persistent map updates.')

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

    def keyframe_scan_callback(self, msg: LaserScan):
        points = []
        for i, r in enumerate(msg.ranges):
            is_hit = (msg.range_min <= r <= msg.range_max) and not math.isnan(r) and not math.isinf(r)
            r_used = r if is_hit else msg.range_max

            angle = msg.angle_min + i * msg.angle_increment
            x_lidar = r_used * math.cos(angle)
            y_lidar = r_used * math.sin(angle)

            x_base = -x_lidar - 0.040
            y_base = -y_lidar

            points.append((x_base, y_base, is_hit))

        self.keyframes.append(points)
        self.get_logger().info(f"Keyframe {len(self.keyframes)-1} received with {len(points)} points")

    def pose_graph_callback(self, msg: PoseArray):
        num_poses = len(msg.poses)
        self.get_logger().info(f"Pose graph update: {num_poses} poses, {len(self.keyframes)} keyframes")

        if num_poses > len(self.keyframes):
            self.get_logger().warning(f"Poses ({num_poses}) exceed keyframes ({len(self.keyframes)}), waiting...")
            return

        # Re-project keyframe scans using updated pose locations
        self.log_odds.fill(0.0)
        for idx in range(min(num_poses, len(self.keyframes))):
            pose_msg = msg.poses[idx]

            px = pose_msg.position.x
            py = pose_msg.position.y
            qx = pose_msg.orientation.x
            qy = pose_msg.orientation.y
            qz = pose_msg.orientation.z
            qw = pose_msg.orientation.w

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            ptheta = math.atan2(siny_cosp, cosy_cosp)

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

                # Endpoint updating
                if self.in_bounds(egx, egy):
                    if is_hit:
                        self.log_odds[egy, egx] = np.clip(
                            self.log_odds[egy, egx] + self.l_occ - self.l0, self.l_min, self.l_max
                        )
                    else:
                        self.log_odds[egy, egx] = np.clip(
                            self.log_odds[egy, egx] + self.l_free - self.l0, self.l_min, self.l_max
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

        clamped_odds = np.clip(self.log_odds, -10, 10)
        prob = 1.0 - 1.0 / (1.0 + np.exp(clamped_odds))

        data = np.full(self.log_odds.shape, -1, dtype=np.int8)
        known = self.log_odds != 0.0
        data[known] = np.clip((prob[known] * 100.0).astype(np.int8), 0, 100)

        grid_msg.data = data.flatten().tolist()
        self.map_pub.publish(grid_msg)

        num_occupied = np.sum(data > 50)
        num_free = np.sum((data < 50) & (data >= 0))
        num_unknown = np.sum(data == -1)
        if num_occupied > 0 or num_free > 0:
            self.get_logger().info(f"Map: {num_occupied} occupied, {num_free} free, {num_unknown} unknown cells")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()