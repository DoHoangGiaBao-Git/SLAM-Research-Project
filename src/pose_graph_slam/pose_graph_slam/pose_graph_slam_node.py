#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped, PoseArray, Pose as RosPose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import math
import numpy as np

# Pose data structure
class Pose:
    def __init__(self, map_x, map_y, map_theta, odom_x, odom_y, odom_theta, scan_msg):
        self.x = map_x
        self.y = map_y
        self.theta = map_theta

        self.odom_x = odom_x
        self.odom_y = odom_y
        self.odom_theta = odom_theta

        self.scan = scan_msg

# Constraint data structure
class Constraints:
    def __init__(self, from_id, to_id, dx, dy, dtheta, is_loop=False):
        self.from_id = from_id
        self.to_id = to_id
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta
        self.is_loop = is_loop

class PoseGraphSLAMNode(Node):
    def __init__(self):
        super().__init__('pose_graph_slam_node')

        # === Parameters ===
        self.min_dist = 0.5            # Minimum distance to save a pose
        self.min_angle = 0.785         # Minimum angle to save a pose

        self.poses = []                 # List of Poses (Nodes)
        self.constraints = []           # List of Constraints (Edges)

        self.current_scan = None        # Store points for extraction (in base_link frame)
        self.latest_scan_msg = None     # Store raw message for keyframe publishing
        self.min_poses = 8              # Check loop closure after 8 poses
        self.min_loop_travel = 1.0      # Need 1m travel between loop attempts
        self.icp_threshold = 4.0        # Candidate search radius in meters
        self.icp_fitness_threshold = 0.15 # Max mean inlier distance in meters
        self.icp_max_correspondence_dist = 0.3 # Correspondence distance
        self.last_loop_closure_id = -1

        # === TF Buffer & Listener ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === TF Broadcaster & Publishers ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/pose_graph_markers', 10)
        self.pose_array_pub = self.create_publisher(PoseArray, '/pose_graph_nodes', 10)
        self.keyframe_scan_pub = self.create_publisher(LaserScan, '/keyframe_scan', 10)

        # === Subscriptions ===
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Continuous 10 Hz timer to publish map->odom TF
        self.tf_timer = self.create_timer(0.1, self.publish_map_to_odom_tf)

        self.get_logger().info("Pose Graph SLAM Node active with robust loop closure detection.")

    def publish_pose_array(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for p in self.poses:
            pose = RosPose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.orientation.z = math.sin(p.theta / 2.0)
            pose.orientation.w = math.cos(p.theta / 2.0)
            msg.poses.append(pose)
        self.pose_array_pub.publish(msg)

    def icp(self, source_pts, target_pts, max_iterations=25, init_guess=(0.0, 0.0, 0.0)):
        src = np.array(source_pts)
        tgt = np.array(target_pts)

        if len(src) < 15 or len(tgt) < 15:
            return 0.0, 0.0, 0.0, float('inf')

        init_dx, init_dy, init_dtheta = init_guess
        R = np.array([[math.cos(init_dtheta), -math.sin(init_dtheta)],
                      [math.sin(init_dtheta), math.cos(init_dtheta)]])
        t = np.array([init_dx, init_dy])
        src = (R @ src.T).T + t

        for _ in range(max_iterations):
            diffs = src[:, None, :] - tgt[None, :, :]
            dists = np.linalg.norm(diffs, axis=2)
            nn_idx = np.argmin(dists, axis=1)
            nn_dist = dists[np.arange(len(src)), nn_idx]
            matched_tgt = tgt[nn_idx]

            inliers = nn_dist <= self.icp_max_correspondence_dist
            num_inliers = np.count_nonzero(inliers)
            if num_inliers < 10:
                break

            centroid_src = np.mean(src[inliers], axis=0)
            centroid_tgt = np.mean(matched_tgt[inliers], axis=0)

            src_centered = src[inliers] - centroid_src
            tgt_centered = matched_tgt[inliers] - centroid_tgt

            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R_step = Vt.T @ U.T

            if np.linalg.det(R_step) < 0:
                Vt[1, :] *= -1
                R_step = Vt.T @ U.T

            t_step = centroid_tgt - (R_step @ centroid_src)

            src = (R_step @ src.T).T + t_step
            R = R_step @ R
            t = R_step @ t + t_step

        dx = t[0]
        dy = t[1]
        dtheta = np.arctan2(R[1, 0], R[0, 0])

        diffs = src[:, None, :] - tgt[None, :, :]
        final_dists = np.min(np.linalg.norm(diffs, axis=2), axis=1)
        inlier_dists = final_dists[final_dists <= self.icp_max_correspondence_dist]
        
        if len(inlier_dists) < 20:
            fitness = float('inf')
        else:
            fitness = float(np.mean(inlier_dists))

        return dx, dy, dtheta, fitness

    def graph_optimization(self, iterations=10):
        num_nodes = len(self.poses)
        if num_nodes < 2 or not self.constraints:
            return

        Omega = np.diag([10.0, 10.0, 5.0])

        for step in range(iterations):
            H = np.zeros((3 * num_nodes, 3 * num_nodes))
            b = np.zeros(3 * num_nodes)

            for constraint in self.constraints:
                i = constraint.from_id
                j = constraint.to_id

                node_i = self.poses[i]
                node_j = self.poses[j]

                cos_i = math.cos(node_i.theta)
                sin_i = math.sin(node_i.theta)
                dx_global = node_j.x - node_i.x
                dy_global = node_j.y - node_i.y

                hat_dx = cos_i * dx_global + sin_i * dy_global
                hat_dy = -sin_i * dx_global + cos_i * dy_global
                hat_dtheta = math.atan2(math.sin(node_j.theta - node_i.theta), math.cos(node_j.theta - node_i.theta))

                e = np.array([
                    hat_dx - constraint.dx,
                    hat_dy - constraint.dy,
                    math.atan2(math.sin(hat_dtheta - constraint.dtheta), math.cos(hat_dtheta - constraint.dtheta))
                ])

                A = np.array([[-cos_i, -sin_i, hat_dy], [sin_i, -cos_i, -hat_dx], [0.0, 0.0, -1.0]])
                B = np.array([[cos_i, sin_i, 0.0], [-sin_i, cos_i, 0.0], [0.0, 0.0, 1.0]])

                idx_i = slice(3 * i, 3 * i + 3)
                idx_j = slice(3 * j, 3 * j + 3)

                H[idx_i, idx_i] += A.T @ Omega @ A
                H[idx_i, idx_j] += A.T @ Omega @ B
                H[idx_j, idx_i] += B.T @ Omega @ A
                H[idx_j, idx_j] += B.T @ Omega @ B

                b[idx_i] += A.T @ Omega @ e
                b[idx_j] += B.T @ Omega @ e

            # Anchor node 0
            H[0:3, 0:3] += np.eye(3) * 1e9

            delta = np.linalg.solve(H, -b)

            for k in range(num_nodes):
                self.poses[k].x += delta[3 * k]
                self.poses[k].y += delta[3 * k + 1]
                self.poses[k].theta += delta[3 * k + 2]
                self.poses[k].theta = math.atan2(math.sin(self.poses[k].theta), math.cos(self.poses[k].theta))

            if np.linalg.norm(delta) < 1e-4:
                break

        self.get_logger().info(f"Pose graph optimization complete over {num_nodes} nodes.")
        self.publish_map_to_odom_tf()
        self.publish_pose_array()

    def check_loop_closure(self):
        if len(self.poses) < self.min_poses:
            return

        current_id = len(self.poses) - 1
        current_node = self.poses[current_id]

        if self.last_loop_closure_id >= 0:
            dist_since_last = math.hypot(
                current_node.x - self.poses[self.last_loop_closure_id].x,
                current_node.y - self.poses[self.last_loop_closure_id].y
            )
            if dist_since_last < self.min_loop_travel:
                return

        best_candidate = None
        best_fitness = float('inf')
        best_transform = None

        for candidate_id in range(current_id - self.min_poses):
            candidate_node = self.poses[candidate_id]
            dist = math.hypot(current_node.x - candidate_node.x, current_node.y - candidate_node.y)

            if dist > self.icp_threshold:
                continue

            if self.last_loop_closure_id >= 0 and abs(candidate_id - self.last_loop_closure_id) < 5:
                continue

            cos_c = math.cos(candidate_node.theta)
            sin_c = math.sin(candidate_node.theta)
            dx_global = current_node.x - candidate_node.x
            dy_global = current_node.y - candidate_node.y
            init_dx = cos_c * dx_global + sin_c * dy_global
            init_dy = -sin_c * dx_global + cos_c * dy_global
            init_dtheta = math.atan2(math.sin(current_node.theta - candidate_node.theta),
                                     math.cos(current_node.theta - candidate_node.theta))

            dx, dy, dtheta, fitness = self.icp(
                current_node.scan, candidate_node.scan, init_guess=(init_dx, init_dy, init_dtheta)
            )

            if fitness > self.icp_fitness_threshold:
                continue

            # Validate that ICP correction from initial guess is bounded
            corr_dist = math.hypot(dx - init_dx, dy - init_dy)
            corr_dtheta = abs(math.atan2(math.sin(dtheta - init_dtheta), math.cos(dtheta - init_dtheta)))

            if corr_dist < 1.0 and corr_dtheta < 0.6:
                if fitness < best_fitness:
                    best_fitness = fitness
                    best_candidate = candidate_id
                    best_transform = (dx, dy, dtheta)

        if best_candidate is not None:
            dx, dy, dtheta = best_transform
            virtual_edge = Constraints(best_candidate, current_id, dx, dy, dtheta, is_loop=True)
            self.constraints.append(virtual_edge)
            self.last_loop_closure_id = current_id
            self.get_logger().info(
                f"LOOP CLOSURE DETECTED: Node {best_candidate} <-> Node {current_id} (fitness={best_fitness:.4f})"
            )
            self.graph_optimization()

    def publish_map_to_odom_tf(self):
        if not self.poses:
            return

        last_node = self.poses[-1]
        dtheta = math.atan2(math.sin(last_node.theta - last_node.odom_theta),
                            math.cos(last_node.theta - last_node.odom_theta))

        cos_t = math.cos(dtheta)
        sin_t = math.sin(dtheta)

        tx = last_node.x - (cos_t * last_node.odom_x - sin_t * last_node.odom_y)
        ty = last_node.y - (sin_t * last_node.odom_x + cos_t * last_node.odom_y)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(dtheta / 2.0)
        t.transform.rotation.w = math.cos(dtheta / 2.0)

        self.tf_broadcaster.sendTransform(t)

    def publish_visualizations(self):
        marker_array = MarkerArray()

        nodes_marker = Marker()
        nodes_marker.header.frame_id = "map"
        nodes_marker.header.stamp = self.get_clock().now().to_msg()
        nodes_marker.ns = "nodes"
        nodes_marker.id = 0
        nodes_marker.type = Marker.TRIANGLE_LIST
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x, nodes_marker.scale.y, nodes_marker.scale.z = 1.0, 1.0, 1.0
        nodes_marker.color.a = 1.0
        nodes_marker.color.b = 1.0

        L, W = 0.25, 0.15
        for pose in self.poses:
            cos_t = math.cos(pose.theta)
            sin_t = math.sin(pose.theta)
            p_front = Point(x=pose.x + L * cos_t, y=pose.y + L * sin_t, z=0.0)
            p_left = Point(x=pose.x - (L / 2.0) * cos_t - (W / 2.0) * sin_t, y=pose.y - (L / 2.0) * sin_t + (W / 2.0) * cos_t, z=0.0)
            p_right = Point(x=pose.x - (L / 2.0) * cos_t + (W / 2.0) * sin_t, y=pose.y - (L / 2.0) * sin_t - (W / 2.0) * cos_t, z=0.0)
            nodes_marker.points.extend([p_front, p_left, p_right])

        marker_array.markers.append(nodes_marker)

        seq_marker = Marker()
        seq_marker.header.frame_id = "map"
        seq_marker.header.stamp = self.get_clock().now().to_msg()
        seq_marker.ns = "seq_edges"
        seq_marker.id = 1
        seq_marker.type = Marker.LINE_LIST
        seq_marker.action = Marker.ADD
        seq_marker.scale.x = 0.03
        seq_marker.color.a, seq_marker.color.g = 1.0, 1.0

        loop_marker = Marker()
        loop_marker.header.frame_id = "map"
        loop_marker.header.stamp = self.get_clock().now().to_msg()
        loop_marker.ns = "loop_edges"
        loop_marker.id = 2
        loop_marker.type = Marker.LINE_LIST
        loop_marker.action = Marker.ADD
        loop_marker.scale.x = 0.05
        loop_marker.color.a, loop_marker.color.r = 1.0, 1.0

        for constraint in self.constraints:
            p1 = Point(x=self.poses[constraint.from_id].x, y=self.poses[constraint.from_id].y, z=0.0)
            p2 = Point(x=self.poses[constraint.to_id].x, y=self.poses[constraint.to_id].y, z=0.0)
            if constraint.is_loop:
                loop_marker.points.extend([p1, p2])
            else:
                seq_marker.points.extend([p1, p2])

        marker_array.markers.append(seq_marker)
        marker_array.markers.append(loop_marker)
        self.marker_pub.publish(marker_array)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan_msg = msg
        points = []

        # Transform laser scan points from laser frame to base_link frame using TF
        # This ensures correct frame transformation regardless of URDF configuration
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                msg.header.frame_id,  # source frame (laser)
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed for scan: {e}')
            return

        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or math.isnan(r) or math.isinf(r):
                continue
            angle = msg.angle_min + (i * msg.angle_increment)
            x_lidar = r * math.cos(angle)
            y_lidar = r * math.sin(angle)

            # Create point in laser frame
            point_laser = PointStamped()
            point_laser.header = msg.header
            point_laser.point.x = x_lidar
            point_laser.point.y = y_lidar
            point_laser.point.z = 0.0

            # Transform to base_link frame
            try:
                point_base = self.tf_buffer.transform(point_laser, 'base_link', timeout=rclpy.duration.Duration(seconds=0.05))
                points.append((point_base.point.x, point_base.point.y))
            except Exception as e:
                self.get_logger().debug(f'Point transform failed: {e}')
                continue

        self.current_scan = points

    def odom_callback(self, msg: Odometry):
        if self.current_scan is None or self.latest_scan_msg is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if not self.poses:
            first_node = Pose(x, y, theta, x, y, theta, self.current_scan)
            self.poses.append(first_node)
            self.keyframe_scan_pub.publish(self.latest_scan_msg)
            self.get_logger().info(f"Node 0 initialized | x={x:.3f}, y={y:.3f}")
            self.publish_map_to_odom_tf()
            self.publish_pose_array()
            return

        last_node = self.poses[-1]

        global_dx = x - last_node.odom_x
        global_dy = y - last_node.odom_y
        dtheta = math.atan2(math.sin(theta - last_node.odom_theta), math.cos(theta - last_node.odom_theta))
        dist = math.hypot(global_dx, global_dy)

        if dist >= self.min_dist or abs(dtheta) >= self.min_angle:
            cos_odom = math.cos(last_node.odom_theta)
            sin_odom = math.sin(last_node.odom_theta)
            local_dx = cos_odom * global_dx + sin_odom * global_dy
            local_dy = -sin_odom * global_dx + cos_odom * global_dy

            cos_map = math.cos(last_node.theta)
            sin_map = math.sin(last_node.theta)
            map_x = last_node.x + (cos_map * local_dx - sin_map * local_dy)
            map_y = last_node.y + (sin_map * local_dx + cos_map * local_dy)
            map_theta = math.atan2(math.sin(last_node.theta + dtheta), math.cos(last_node.theta + dtheta))

            new_node = Pose(map_x, map_y, map_theta, x, y, theta, self.current_scan)
            self.poses.append(new_node)
            self.keyframe_scan_pub.publish(self.latest_scan_msg)

            from_id = len(self.poses) - 2
            to_id = len(self.poses) - 1
            new_edge = Constraints(from_id, to_id, local_dx, local_dy, dtheta)
            self.constraints.append(new_edge)

            self.check_loop_closure()
            self.publish_visualizations()
            self.publish_pose_array()

        self.publish_map_to_odom_tf()

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()