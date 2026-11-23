#!/usr/bin/env python3
import math
import time

import numpy as np
from PIL import Image
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, Imu
import yaml

class OccupancyGridMapV2Node(Node):
    def __init__(self):
        super().__init__('occupancy_grid_map_v2_node')

        # Expanded grid parameters to prevent left/bottom clipping
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 1600)       # Expanded from 800 (80m total span)
        self.declare_parameter('map_height', 1600)      # Expanded from 800 (80m total span)
        self.declare_parameter('map_origin_x', -40.0)   # Expanded from -20.0 to shift grid origin left
        self.declare_parameter('map_origin_y', -40.0)   # Expanded from -20.0
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('map_frame', 'map')
        # NEW: pose change that forces a full rebuild, and rebuild throttle
        self.declare_parameter('rebuild_shift_m', 0.0)
        self.declare_parameter('rebuild_shift_rad', 0.0)
        self.declare_parameter('min_rebuild_period', 2.0)
        self.declare_parameter('max_angular_vel', 0.25)
        self.declare_parameter('do_save_map_to_disk', False)

        self.resolution = float(self.get_parameter('map_resolution').value)
        self.width = int(self.get_parameter('map_width').value)
        self.height = int(self.get_parameter('map_height').value)
        self.origin_x = float(self.get_parameter('map_origin_x').value)
        self.origin_y = float(self.get_parameter('map_origin_y').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.rebuild_shift_m = float(self.get_parameter('rebuild_shift_m').value)
        self.rebuild_shift_rad = float(self.get_parameter('rebuild_shift_rad').value)
        self.min_rebuild_period = float(self.get_parameter('min_rebuild_period').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.do_save_map_to_disk = bool(self.get_parameter('do_save_map_to_disk').value)

        if self.resolution <= 0.0 or self.width <= 0 or self.height <= 0 or publish_rate <= 0.0:
            raise ValueError('Map resolution, dimensions, and publish rate must be positive')

        # Fixed laser-to-base_link transform from robot_model/urdf/lidar.xacro.
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', math.pi / 2)
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)
        self.laser_yaw = float(self.get_parameter('laser_yaw').value)

        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.free_update = -0.2
        self.occupied_update = 0.85
        self.min_log_odds = -5.0
        self.max_log_odds = 5.0

        self.scans = []
        self.poses = None
        self.map_dirty = False
        self.map_msg = None
        self.last_rebuild_time = -1e9

        self.last_processed_keyframe = 0

        self.current_angular_vel = 0.0

        # QoS profile matching pose_graph_slam node for reliable transient_local keyframe scans
        keyframe_qos = QoSProfile(
            depth=1000,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(LaserScan, 'slam/keyframe_scan', self.scan_callback, keyframe_qos)
        self.create_subscription(PoseArray, 'slam/poses', self.poses_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.create_timer(1.0 / publish_rate, self.publish_map)

        self.get_logger().info('Occupancy grid mapper node is ready.')

    @staticmethod
    def quaternion_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def world_to_grid(self, x, y):
        gx = math.floor((x - self.origin_x) / self.resolution)
        gy = math.floor((y - self.origin_y) / self.resolution)
        return gx, gy

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        x, y = x0, y0

        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                return cells
            twice_error = 2 * error
            if twice_error >= dy:
                error += dy
                x += sx
            if twice_error <= dx:
                error += dx
                y += sy

    def scan_callback(self, msg):
        self.scans.append(msg)
        # Incremental processing without triggering heavy full-map rebuilds
        self.process_incremental_scans()

    def poses_callback(self, msg):
        new_poses = [
            (pose.position.x, pose.position.y, self.quaternion_yaw(pose.orientation))
            for pose in msg.poses
        ]

        # Rebuild full map ONLY when pose graph optimization shifts historical keyframe poses.
        # CHANGED: compare the COMMON PREFIX. Loop closure is detected in the same
        # callback that appends a new keyframe, so the optimized PoseArray is always
        # one element longer than the previous one; the old `len(new) == len(old)`
        # test therefore skipped exactly the messages that carry the correction, and
        # the map was never rebuilt (stale walls next to the live scan).
        if self.poses is not None:
            n = min(len(new_poses), len(self.poses))
            for p1, p2 in zip(new_poses[:n], self.poses[:n]):
                shift = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                dyaw = abs(math.atan2(math.sin(p1[2] - p2[2]), math.cos(p1[2] - p2[2])))
                if shift > self.rebuild_shift_m or dyaw > self.rebuild_shift_rad:
                    self.map_dirty = True
                    break

        self.poses = new_poses
        self.process_incremental_scans()

    def process_incremental_scans(self):
        if self.map_dirty:
            return

        if not self.poses or not self.scans:
            return

        updated = False
        while self.last_processed_keyframe < min(len(self.poses), len(self.scans)):
            idx = self.last_processed_keyframe
            self.add_scan_to_map(self.scans[idx], self.poses[idx])
            self.last_processed_keyframe += 1
            updated = True

        if updated:
            self.update_occupancy_grid_msg()

    def add_scan_to_map(self, scan, pose):
        pose_x, pose_y, pose_yaw = pose
        cos_pose = math.cos(pose_yaw)
        sin_pose = math.sin(pose_yaw)

        origin_x = pose_x + cos_pose * self.laser_x - sin_pose * self.laser_y
        origin_y = pose_y + sin_pose * self.laser_x + cos_pose * self.laser_y
        start_x, start_y = self.world_to_grid(origin_x, origin_y)

        for index, measured_range in enumerate(scan.ranges):
            # Only use real returns; no-return rays tell us nothing about free space
            if not math.isfinite(measured_range) or measured_range < scan.range_min or measured_range >= scan.range_max:
                continue

            angle = pose_yaw + self.laser_yaw + scan.angle_min + index * scan.angle_increment
            end_x = origin_x + measured_range * math.cos(angle)
            end_y = origin_y + measured_range * math.sin(angle)
            grid_end_x, grid_end_y = self.world_to_grid(end_x, end_y)
            ray = self.bresenham(start_x, start_y, grid_end_x, grid_end_y)

            # Free space along the ray
            for gx, gy in ray[:-1]:
                if self.in_bounds(gx, gy):
                    self.log_odds[gy, gx] = max(self.min_log_odds, self.log_odds[gy, gx] + self.free_update)

            # Occupied endpoint
            gx, gy = ray[-1]
            if self.in_bounds(gx, gy):
                self.log_odds[gy, gx] = min(self.max_log_odds, self.log_odds[gy, gx] + self.occupied_update)

    def update_occupancy_grid_msg(self):
        """Fast vectorized generation of ROS OccupancyGrid message from current log odds"""
        occupancy = np.full((self.height, self.width), -1, dtype=np.int8)

        free_mask = self.log_odds < -0.2
        occ_mask = self.log_odds > 0.2

        # Free space = 0 (White), Occupied = 50..100 (Gray to Black)
        occupancy[free_mask] = 0
        prob_occ = 1.0 / (1.0 + np.exp(-self.log_odds[occ_mask]))
        occupancy[occ_mask] = np.clip(prob_occ * 100.0, 50.0, 100.0).astype(np.int8)

        self.map_msg = OccupancyGrid()
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.info.origin.position.x = self.origin_x
        self.map_msg.info.origin.position.y = self.origin_y
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.info.map_load_time = self.get_clock().now().to_msg()
        # CHANGED: hand rclpy the int8 ndarray directly; .tolist() on 2.56M cells
        # allocated a huge Python list on every update and stalled the executor.
        self.map_msg.data = occupancy.ravel()

    def rebuild_map(self):
        """Full O(N) map rebuild pass executed strictly after loop closure optimization"""
        if not self.poses or not self.scans:
            return False

        num_keyframes = min(len(self.poses), len(self.scans))

        self.log_odds.fill(0.0)
        for i in range(num_keyframes):
            self.add_scan_to_map(self.scans[i], self.poses[i])

        self.last_processed_keyframe = num_keyframes
        self.update_occupancy_grid_msg()
        self.map_dirty = False
        self.last_rebuild_time = time.monotonic()
        self.get_logger().info(f'Rebuilt full map from {num_keyframes} keyframes after loop closure.')
        return True

    def publish_map(self):
        # CHANGED: throttle rebuilds (pure-Python raycasting is slow for many keyframes)
        if self.map_dirty and (time.monotonic() - self.last_rebuild_time) >= self.min_rebuild_period:
            self.rebuild_map()
        if self.map_msg is None:
            return

        now = self.get_clock().now().to_msg()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.header.stamp = now
        self.map_pub.publish(self.map_msg)

    def save_map_to_disk(self, filename="custom_slam_map"):
        # Convert log-odds directly to occupancy image
        free_mask = self.log_odds < -0.2
        occ_mask = self.log_odds > 0.2
        img_data = np.full(self.log_odds.shape, 205, dtype=np.uint8)
        img_data[free_mask] = 255
        img_data[occ_mask] = 0
        img_data = np.flipud(img_data)

        Image.fromarray(img_data).save(f"{filename}.pgm")
        yaml_data = {
            'image': f"{filename}.pgm",
            'resolution': self.resolution,
            'origin': [self.origin_x, self.origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(f"{filename}.yaml", 'w') as f:
            yaml.dump(yaml_data, f)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.do_save_map_to_disk:
            node.save_map_to_disk()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
