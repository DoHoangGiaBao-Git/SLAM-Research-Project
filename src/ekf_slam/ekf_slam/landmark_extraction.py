#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np

class PointLandmarkExtractionNode(Node):
    def __init__(self):
        super().__init__('landmark_extraction_node')
        # Constants
        self.distance_threshold = 0.2
        self.line_distance_threshold = 0.1
        self.min_cluster_size = 5

        # Subscriber and publisher
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/landmark_markers', 
            10
        )

        
    def scan_callback(self, msg: LaserScan):
        # Convert LaserScan to Point Cloud
        points = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            points.append((x, y))

        if not points:
            return
        
        # Segmentation / Clustering
        clusters = self.segment_points(points)

        # Feature Extraction (Split-and-Merge Corner Detection)
        landmarks = []
        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                extract_corners = self.split_and_merge(cluster)
                landmarks.extend(extract_corners)

        # Post-filter: split-and-merge alone tends to plant spurious "corners"
        # on straight walls from sensor noise, and duplicate corners when two
        # clusters overlap. Merge anything closer than merge_radius together.
        landmarks = self.merge_close_landmarks(landmarks, merge_radius=0.15)

        self.publish_markers(landmarks, msg.header.frame_id, msg.header.stamp)

    def segment_points(self, points):
        # Group continous point ranges into separate clusters
        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            p1 = points[i - 1]
            p2 = points[i]

            dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

            if dist < self.distance_threshold:
                current_cluster.append(p2)
            else:
                clusters.append(current_cluster)
                current_cluster = [p2]

        if current_cluster:
            clusters.append(current_cluster)

        return clusters
    
    def split_and_merge(self, cluster):
        # Apply Split-and-Merge algorithm to find corner points
        if len(cluster) < 3:
            return []
        
        p_first = cluster[0]
        p_last = cluster[-1]

        max_dist = 0
        max_idx = -1

        for i in range(1, len(cluster) - 1):
            d = self.perpendicular_distance(cluster[i], p_first, p_last)
            if d > max_dist:
                max_dist = d
                max_idx = i
            
        if max_dist > self.line_distance_threshold:
            left_corners = self.split_and_merge(cluster[:max_idx + 1])
            right_corners = self.split_and_merge(cluster[max_idx:])
            corner_point = (float(cluster[max_idx][0]), float(cluster[max_idx][1]))

            return left_corners + [corner_point] + right_corners
            
        return []
    
    @staticmethod
    def merge_close_landmarks(landmarks, merge_radius=0.15):
        # Collapse any landmarks within merge_radius of each other into a
        # single point (their centroid). Cheap O(n^2) is fine since n is
        # small (a handful of corners per scan).
        merged = []
        used = [False] * len(landmarks)

        for i, lm_i in enumerate(landmarks):
            if used[i]:
                continue
            group = [lm_i]
            used[i] = True
            for j in range(i + 1, len(landmarks)):
                if used[j]:
                    continue
                lm_j = landmarks[j]
                dist = math.hypot(lm_i[0] - lm_j[0], lm_i[1] - lm_j[1])
                if dist < merge_radius:
                    group.append(lm_j)
                    used[j] = True
            cx = sum(p[0] for p in group) / len(group)
            cy = sum(p[1] for p in group) / len(group)
            merged.append((cx, cy))

        return merged

    @staticmethod
    def perpendicular_distance(pt, line_start, line_end):
        # Calculates perpendicular distance of a point to a line segment.
        x0, y0 = pt
        x1, y1 = line_start
        x2, y2 = line_end

        num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        den = math.hypot(y2 - y1, x2 - x1)

        if den == 0:
            return math.hypot(x0 - x1, y0 - y1)
        
        return num / den
        
    def publish_markers(self, landmarks, frame_id, timestamp):
        marker_array = MarkerArray()

        # Delete previous markers in Rviz to prevent ghost markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for idx, lm in enumerate(landmarks):
            if not (isinstance(lm, (tuple, list, np.ndarray)) and len(lm) >= 2):
                continue

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = timestamp
            marker.ns = "landmarks"
            marker.id = idx  # Unique ID per landmark marker
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Marker Position
            marker.pose.position.x = float(lm[0])
            marker.pose.position.y = float(lm[1])
            marker.pose.position.z = 0.0

            # Orientation default (identity quaternion)
            marker.pose.orientation.w = 1.0

            # Scale
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            
            # Color: Bright Blue
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
            

def main(args=None):
    rclpy.init(args=args)
    node = PointLandmarkExtractionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()