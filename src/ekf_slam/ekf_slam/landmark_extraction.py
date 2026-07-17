#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
import numpy as np

class PointLandmarkExtractionNode(Node):
    def __init__(self):
        super().__init__('point_landmark_extraction_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            'robot1/scan',
            self.scan_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker, 
            'robot1/landmark_markers', 
            10
        )
        
        # Tuning parameters
        self.distance_threshold = 0.1  # Maximum split deviation (meters)
        self.N = 10                     # Min points for a valid wall segment
        self.min_corner_angle = 45.0   # Min angle (degrees) to qualify as a corner
        self.max_corner_angle = 135.0  # Max angle (degrees) to qualify as a corner

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Convert Polar to Cartesian
        points = []
        for i, r in enumerate(ranges):
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                continue
            angle = angle_min + i * angle_increment
            points.append([r * math.cos(angle), r * math.sin(angle)])
        
        if len(points) < self.N:
            return
        
        points = np.array(points)

        # Extract corner landmarks
        corners = self.extract_corners(points)
        
        self.get_logger().info(f"Extracted {len(corners)} point landmarks for EKF-SLAM.")

        # Publish the landmarks as visualization markers
        self.publish_corners_rviz(corners, frame_id=msg.header.frame_id)

    def get_perpendicular_distance(self, p, p1, p2):
        numerator = np.abs((p2[1] - p1[1]) * p[0] - (p2[0] - p1[0]) * p[1] + p2[0] * p1[1] - p2[1] * p1[0])
        denominator = np.sqrt((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)
        return numerator / denominator if denominator != 0 else np.linalg.norm(p - p1)

    def extract_corners(self, points):
        """
        Runs a simplified split-based algorithm that collects the sharp 
        intersections (vertices) between segments.
        """
        stack = [(0, len(points) - 1)]
        split_indices = []

        # 1. Split and find all significant vertices (break points)
        while len(stack) > 0:
            start_idx, end_idx = stack.pop(0)
            if (end_idx - start_idx) < self.N:
                continue

            p_start = points[start_idx]
            p_end = points[end_idx]

            max_dist = 0.0
            split_idx = -1

            for i in range(start_idx + 1, end_idx):
                dist = self.get_perpendicular_distance(points[i], p_start, p_end)
                if dist > max_dist:
                    max_dist = dist
                    split_idx = i

            if max_dist > self.distance_threshold:
                split_indices.append(split_idx)
                # Keep splitting both sides
                stack.append((start_idx, split_idx))
                stack.append((split_idx, end_idx))

        # 2. Filter vertices to keep only structural geometric corners
        corners = []
        split_indices.sort()

        for idx in split_indices:
            # Look at a few points before and after the split to calculate vectors
            if idx - 5 < 0 or idx + 5 >= len(points):
                continue
                
            # Vector 1 (leading into the corner) and Vector 2 (leaving the corner)
            v1 = points[idx] - points[idx - 5]
            v2 = points[idx + 5] - points[idx]

            # Calculate the angle between the two vectors
            cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
            angle = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))

            # If the vectors change direction sharply, it's a structural corner (e.g., ~90 degrees)
            if self.min_corner_angle <= angle <= self.max_corner_angle:
                corners.append(points[idx].tolist())

        return corners
    
    def publish_corners_rviz(self, corners, frame_id="base_link"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "landmarks"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Size of the points
        marker.scale.y = 0.1
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 1.0  # Blue

        for corner in corners:
            point = Point()
            point.x, point.y, point.z = corner[0], corner[1], 0.0
            marker.points.append(point)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PointLandmarkExtractionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()