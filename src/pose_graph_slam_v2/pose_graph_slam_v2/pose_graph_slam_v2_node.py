#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PoseArray, Pose as GeoPose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import math
import time
from collections import deque
import numpy as np
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree
from concurrent.futures import ThreadPoolExecutor

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

class Pose:
    """Represent a Graph Node"""
    def __init__(self, node_id, map_x, map_y, map_theta, odom_x, odom_y, odom_theta, scan_points, stamp_sec=0.0):
        # Pose ID
        self.id = node_id
        self.stamp = stamp_sec

        # Pose estimate in map frame
        self.x = map_x
        self.y = map_y
        self.theta = map_theta

        # Pose estimate in odom frame
        self.odom_x = odom_x
        self.odom_y = odom_y
        self.odom_theta = odom_theta

        # Local Cartesian scan points (base_link frame)
        self.scan_points = scan_points

class Constraint:
    """Represent a Graph Edge (Transformation between two nodes)"""
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

        # Graph Storage
        self.poses = [] # List of Pose objects (Nodes)
        self.constraints = [] # List of Constraint objects (Edges)

        # CHANGED: time-sync buffers replace "latest scan + latest odom" pairing.
        self.odom_buffer = deque(maxlen=500)  # (t, x, y, theta) in odom frame
        self.scan_queue = deque(maxlen=50)    # (t_ref, LaserScan msg, local points)

        # Fixed laser-to-base_link transform from robot_model/urdf/lidar.xacro.
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', math.pi/2)
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)
        self.laser_yaw = float(self.get_parameter('laser_yaw').value)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

        # NEW tunables (defaults are the recommended values)
        self.declare_parameter('require_time_sync', True)   # False = old behaviour (latest odom at scan arrival)
        self.declare_parameter('use_scan_mid_time', True)   # use stamp + scan_time/2 as the scan's reference time
        self.declare_parameter('tf_postdate', 0.1)          # seconds, like slam_toolbox transform_timeout
        self.declare_parameter('frontend_fitness_threshold', 0.05)
        self.declare_parameter('frontend_min_inlier_ratio', 0.5)
        self.declare_parameter('frontend_max_corr_dist', 0.15)
        self.declare_parameter('frontend_max_corr_angle', 0.15)
        self.declare_parameter('loop_fitness_threshold', 0.05)
        self.declare_parameter('loop_min_inlier_ratio', 0.6)
        self.declare_parameter('loop_max_corr_dist', 0.6)
        self.declare_parameter('loop_max_corr_angle', 0.35)
        self.declare_parameter('loop_cauchy_delta', 0.5)
        self.declare_parameter('opt_iterations', 10)
        self.declare_parameter('ror_radius', 0.12)
        self.declare_parameter('ror_min_neighbors', 2)
        self.declare_parameter('max_angular_velocity', 0.25)
        self.declare_parameter('rotation_settle_time', 0.2)
        self.declare_parameter('do_save_trajectory_tum', False)

        self.require_time_sync = bool(self.get_parameter('require_time_sync').value)
        self.use_scan_mid_time = bool(self.get_parameter('use_scan_mid_time').value)
        self.tf_postdate = float(self.get_parameter('tf_postdate').value)
        self.frontend_fitness_threshold = float(self.get_parameter('frontend_fitness_threshold').value)
        self.frontend_min_inlier_ratio = float(self.get_parameter('frontend_min_inlier_ratio').value)
        self.frontend_max_corr_dist = float(self.get_parameter('frontend_max_corr_dist').value)
        self.frontend_max_corr_angle = float(self.get_parameter('frontend_max_corr_angle').value)
        self.loop_fitness_threshold = float(self.get_parameter('loop_fitness_threshold').value)
        self.loop_min_inlier_ratio = float(self.get_parameter('loop_min_inlier_ratio').value)
        self.loop_max_corr_dist = float(self.get_parameter('loop_max_corr_dist').value)
        self.loop_max_corr_angle = float(self.get_parameter('loop_max_corr_angle').value)
        self.loop_cauchy_delta = float(self.get_parameter('loop_cauchy_delta').value)
        self.opt_iterations = int(self.get_parameter('opt_iterations').value)
        self.ror_radius = float(self.get_parameter('ror_radius').value)
        self.ror_min_neighbors = int(self.get_parameter('ror_min_neighbors').value)
        self.do_save_trajectory_tum = bool(self.get_parameter('do_save_trajectory_tum').value)
        self.accum_dist = [0.0] # Tracks cumulative distance along keyframe graph
        self.submap_stride = 4 # Check every 4th keyframe as submap anchor
        self.drift_rate = 0.15 # Dynamic search expansion factor (meters per sqrt(metter traveled))
        self.max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)
        self.settle_time = float(self.get_parameter('rotation_settle_time').value)

        # Parameters for keyframe creation
        self.min_dist = 0.25  # Minimum distance (meters) to create a new keyframe
        self.min_angle = 0.26  # Minimum rotation (raddians, ~15 degrees) to create a new keyframe

        # Parameters for Loop Closure
        self.min_poses = 8 # Check loop closure only after 8 keyframes
        self.min_loop_travel = 1.0 # Require 1m movement between loop attempts
        self.icp_threshold = 4.0 # Candidate search radius (meters)
        self.icp_max_correspondence_dist = 0.3 # Max point pairing distance (meters)
        self.last_loop_closure_id = -1 # Last node ID where loop closure was performed

        # Performance optimization
        self.voxel_size = 0.05  # Voxel grid downsampling (meters)
        self.max_icp_iterations = 20  # Reduced from 25

        # Performance tracking
        self.icp_times = []
        self.loop_closure_times = []
        self.optimization_times = []
        self.last_perf_log = 0

        # Manual submap cache (lru_cache kept stale, half-built submaps)
        self._submap_cache = {}

        # Latest map -> odom estimate, published by a timer
        self.m2o = (0.0, 0.0, 0.0)

        # TF Broadcaster for map -> odom transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # ThreadPoolWorker for graph optimization
        self._optimizer_pool = ThreadPoolExecutor(max_workers=1)
        self._optimization_future = None
        self._optimization_snapshot = None
        self._optimization_pending = False

        # Current angular velocity
        self.current_angular_vel = 0.0

        # Time when the last high angular velocity was detected
        self.last_high_rotation_time = 0.0

        self.create_timer(0.02, self._poll_optimizer)

        # Subscriptions
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, 'slam/path', 10) # Odometry Path
        self.graph_pub = self.create_publisher(MarkerArray, 'slam/pose_graph', 10) # Loop Closure
        self.poses_pub = self.create_publisher(PoseArray, 'slam/poses', 10) # Robot Poses
        keyframe_qos = QoSProfile(
            depth=1000,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.keyframe_scan_pub = self.create_publisher(LaserScan, 'slam/keyframe_scan', keyframe_qos) # Keyframes LaserScan

        # CHANGED: publish map -> odom at a fixed 50 Hz instead of only when odom arrives
        self.create_timer(0.02, self.publish_map_to_odom_tf)

        self.get_logger().info('Pose Graph SLAM Node has been started.')

    @staticmethod
    def get_yaw_from_quaternion(q):
        """Convert a quaternion into yaw angle (rotation around Z-axis)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def voxel_downsample(self, points, voxel_size=None):
        """Simple voxel grid downsampling to reduce point count"""
        if voxel_size is None:
            voxel_size = self.voxel_size

        if points is None or len(points) == 0:
            return []

        pts = np.array(points)
        if len(pts) == 0:
            return []

        # Quantize to voxel grid
        voxels = {}
        for x, y in pts:
            vx = int(x / voxel_size)
            vy = int(y / voxel_size)
            if (vx, vy) not in voxels:
                voxels[(vx, vy)] = (x, y)

        return list(voxels.values())

    def _invalidate_submap_cache(self):
        """Clear submap cache after pose optimization"""
        self._submap_cache.clear()

    def get_submap(self, anchor_id: int, window_size: int = 3) -> np.ndarray:
        """CHANGED: only cache submaps whose full +/- window already exists.
        The old lru_cache froze submaps built while the newest keyframes were
        still missing, so loop-closure targets were silently half-sized."""
        complete = anchor_id + window_size < len(self.poses)
        key = (anchor_id, window_size)
        if complete and key in self._submap_cache:
            return self._submap_cache[key]
        pts = self.build_submap_cloud(anchor_id, window_size)
        if complete:
            if len(self._submap_cache) > 256:
                self._submap_cache.clear()
            self._submap_cache[key] = pts
        return pts

    def build_submap_cloud(self, anchor_id: int, window_size: int = 3) -> np.ndarray:
        """Aggregate scans from [anchor_id - window_size, anchor_id + window_size] and transform them into local map frame of anchor node"""
        anchor_node = self.poses[anchor_id]
        cos_k = math.cos(anchor_node.theta)
        sin_k = math.sin(anchor_node.theta)

        submap_points = []

        start_idx = max(0, anchor_id - window_size)
        end_idx = min(len(self.poses), anchor_id + window_size + 1)

        for i in range(start_idx, end_idx):
            node = self.poses[i]
            if not node.scan_points:
                continue

            pts = np.array(node.scan_points)
            if len(pts) == 0:
                continue

            # Transform local scan_i points -> Global map frame
            cos_i = math.cos(node.theta)
            sin_i = math.sin(node.theta)
            x_map = node.x + cos_i * pts[:, 0] - sin_i * pts[:, 1]
            y_map = node.y + sin_i * pts[:, 0] + cos_i * pts[:, 1]

            # Transform map frame points -> local anchor_k frame
            dx = x_map - anchor_node.x
            dy = y_map - anchor_node.y
            x_anchor = cos_k * dx + sin_k * dy
            y_anchor = -sin_k * dx + cos_k * dy

            submap_points.append(np.column_stack((x_anchor, y_anchor)))

        if not submap_points:
            return np.empty((0, 2))

        return np.vstack(submap_points)

    def icp(self, source_pts, target_pts, max_iterations=10, init_guess=(0.0, 0.0, 0.0)):
        """SVD-based ICP aligning source_pts (current scan) to target_pts (submap) using cKDTree for fast NN search.
        Returns dx, dy, dtheta, fitness (mean inlier distance), inlier_ratio."""
        icp_start = time.time()
        src = np.array(source_pts)
        tgt = np.array(target_pts)

        if len(src) < 15 or len(tgt) < 15:
            return 0.0, 0.0, 0.0, float('inf'), 0.0  # Not enough points for ICP

        # Build KD-Tree once for target points
        kdtree = cKDTree(tgt)

        # Apply initial guess transformation to source points
        init_dx, init_dy, init_dtheta = init_guess
        R = np.array([
            [math.cos(init_dtheta), -math.sin(init_dtheta)],
            [math.sin(init_dtheta), math.cos(init_dtheta)]
        ])
        t = np.array([init_dx, init_dy])
        src = (R @ src.T).T + t

        for _ in range(max_iterations):
            # Fast nearest neighbor search using KD-Tree
            nn_dist, nn_idx = kdtree.query(src, k=1)

            # Filter by correspondence distance threshold
            inliers = nn_dist <= self.icp_max_correspondence_dist
            if np.count_nonzero(inliers) < 10:
                break  # Not enough inliers to compute a reliable transformation

            # Only use inlier correspondences
            matched_tgt = tgt[nn_idx[inliers]]
            src_inliers = src[inliers]

            centroid_src = np.mean(src_inliers, axis=0)
            centroid_tgt = np.mean(matched_tgt, axis=0)

            src_centered = src_inliers - centroid_src
            tgt_centered = matched_tgt - centroid_tgt

            # SVD for 2D rigid transformation
            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R_step = Vt.T @ U.T

            if np.linalg.det(R_step) < 0:
                Vt[1, :] *= -1
                R_step = Vt.T @ U.T

            t_step = centroid_tgt - R_step @ centroid_src

            # Accumulate transformation
            src = (R_step @ src.T).T + t_step
            R = R_step @ R
            t = R_step @ t + t_step

        dx = t[0]
        dy = t[1]
        dtheta = math.atan2(R[1, 0], R[0, 0])

        # Compute final fitness error (mean inlier distance)
        nn_dist, nn_idx = kdtree.query(src, k=1)
        inlier_mask = nn_dist <= self.icp_max_correspondence_dist
        inlier_dists = nn_dist[inlier_mask]
        inlier_ratio = float(len(inlier_dists)) / float(len(src))

        if len(inlier_dists) < 20:
            fitness = float('inf')  # Not enough inliers for a reliable fitness score
        else:
            fitness = float(np.mean(inlier_dists))

        # Track performance
        icp_time = time.time() - icp_start
        self.icp_times.append(icp_time)

        return dx, dy, dtheta, fitness, inlier_ratio

    def coarse_to_fine_icp(self, src_pts, tgt_pts, init_dx, init_dy, init_dtheta):
        """Evaluates multiple heading orientations on downsampled scans to avoid local minima."""
        src_coarse = self.voxel_downsample(src_pts, voxel_size=0.15)
        tgt_coarse = self.voxel_downsample(tgt_pts, voxel_size=0.15)

        if len(src_coarse) < 10 or len(tgt_coarse) < 10:
            return self.icp(src_pts, tgt_pts, max_iterations=self.max_icp_iterations, init_guess=(init_dx, init_dy, init_dtheta))

        # Test coarse heading offsets (-30 deg to +30 deg)
        yaw_offsets = [-0.52, -0.26, 0.0, 0.26, 0.52]
        best_fitness = float('inf')
        best_guess = (init_dx, init_dy, init_dtheta)

        for offset in yaw_offsets:
            test_yaw = wrap_angle(init_dtheta + offset)
            dx, dy, dth, fitness, inliers = self.icp(
                src_coarse, tgt_coarse, max_iterations=4, init_guess=(init_dx, init_dy, test_yaw)
            )
            if fitness < best_fitness:
                best_fitness = fitness
                best_guess = (dx, dy, dth)

        # Refine winning pose on full-resolution scan
        return self.icp(
            src_pts, tgt_pts, max_iterations=self.max_icp_iterations, init_guess=best_guess
        )

    def check_loop_closure(self):
        """Searches past keyframes for candidate submaps and evaluates loop closure"""
        if len(self.poses) < self.min_poses:
            return

        current_id = len(self.poses) - 1
        current_node = self.poses[current_id]

        # Enforce minium travel distance between loop closure checks
        if self.last_loop_closure_id >= 0:
            dist_since_last = math.hypot(
                current_node.x - self.poses[self.last_loop_closure_id].x,
                current_node.y - self.poses[self.last_loop_closure_id].y
            )
            if dist_since_last < self.min_loop_travel:
                return  # Not enough travel since last loop closure

        best_candidate = None
        best_fitness = float('inf')
        best_transform = None

        # Filter candidates using submap stride and dynamic search radius
        max_candidate_id = current_id - self.min_poses
        for candidate_id in range(0, max_candidate_id, self.submap_stride):
            candidate_node = self.poses[candidate_id]

            # Calculate accumulated travel distance along graph
            traveled_dist = self.accum_dist[current_id] - self.accum_dist[candidate_id]

            # DYNAMIC SEARCH RADIUS: expands proportional to sqrt of traveled distance
            dynamic_icp_threshold = self.icp_threshold + self.drift_rate * math.sqrt(traveled_dist)

            # Spatial Distance Check with Dynamic Radius
            spatial_dist = math.hypot(current_node.x - candidate_node.x, current_node.y - candidate_node.y)
            if spatial_dist > dynamic_icp_threshold:
                continue

            # Skip keyframes adjacent to recent loop closure
            if self.last_loop_closure_id >= 0 and abs(candidate_id - self.last_loop_closure_id) < self.min_poses:
                continue

            # Compute initial relative transform guess
            cos_c = math.cos(candidate_node.theta)
            sin_c = math.sin(candidate_node.theta)
            dx_global = current_node.x - candidate_node.x
            dy_global = current_node.y - candidate_node.y
            init_dx = cos_c * dx_global + sin_c * dy_global
            init_dy = -sin_c * dx_global + cos_c * dy_global
            init_dtheta = wrap_angle(current_node.theta - candidate_node.theta)

            # Build Candidate Submap Target
            submap_target = self.get_submap(candidate_id, window_size=3)

            # Coarse-to-Fine Alignment
            dx, dy, dtheta, fitness, inlier_ratio = self.coarse_to_fine_icp(
                current_node.scan_points, submap_target, init_dx, init_dy, init_dtheta
            )

            if fitness > self.loop_fitness_threshold or inlier_ratio < self.loop_min_inlier_ratio:
                continue

            # Dynamic max allowed correction based on traveled distance
            max_corr_dist = self.loop_max_corr_dist + 0.1 * math.sqrt(traveled_dist)
            max_corr_angle = self.loop_max_corr_angle + 0.05 * math.sqrt(traveled_dist)

            # Ensure ICP alignment correction is bounded relative to initial guess
            corr_dist = math.hypot(dx - init_dx, dy - init_dy)
            corr_dtheta = abs(wrap_angle(dtheta - init_dtheta))

            if corr_dist < max_corr_dist and corr_dtheta < max_corr_angle:
                if fitness < best_fitness:
                    best_fitness = fitness
                    best_candidate = candidate_id
                    best_transform = (dx, dy, dtheta)

        # Add virtual loop closure edge if a match was found
        if best_candidate is not None:
            dx, dy, dtheta = best_transform
            virtual_constraint = Constraint(
                from_id=best_candidate,
                to_id=current_id,
                dx=dx,
                dy=dy,
                dtheta=dtheta,
                is_loop=True
            )
            self.constraints.append(virtual_constraint)
            self.last_loop_closure_id = current_id
            self.get_logger().info(
                f"Loop Closure Detected: Node {current_id} -> Node {best_candidate} | Fitness: {best_fitness:.4f} | Total Edges: {len(self.constraints)}"
            )

            # Execute Graph Optimization after adding a loop closure constraint
            self.request_graph_optimization()

    def request_graph_optimization(self):
        """Request graph optimization to be performed in a separate thread."""
        if self._optimization_future is not None:
            self._optimization_pending = True
            return  # Optimization already in progress; flag for another pass after completion

        poses = tuple(
            (pose.x, pose.y, pose.theta) for pose in self.poses
        )
        constraints = tuple(
            (c.from_id, c.to_id, c.dx, c.dy, c.dtheta, c.is_loop)
            for c in self.constraints
        )

        self._optimization_snapshot = poses
        self._optimization_future = self._optimizer_pool.submit(
            self._optimize_snapshot,
            poses,
            constraints,
            self.opt_iterations,
            self.loop_cauchy_delta
        )
            
    @staticmethod
    def _optimize_snapshot(
        pose_snapshot,
        constraint_snapshot,
        iterations,
        loop_cauchy_delta,
    ):
        """Optimize a private pose copy without accessing ROS node state."""
        poses = np.array(pose_snapshot, dtype=float, copy=True)
        num_nodes = len(poses)

        if num_nodes < 2:
            return poses.tolist()

        for _ in range(iterations):
            H = lil_matrix((3 * num_nodes, 3 * num_nodes))
            b = np.zeros(3 * num_nodes)

            # Anchor root node (Node 0) to fix the global frame.
            H[0:3, 0:3] += np.eye(3) * 1e6

            for i, j, meas_dx, meas_dy, meas_dtheta, is_loop in constraint_snapshot:
                node_i = poses[i]
                node_j = poses[j]

                cos_i = math.cos(node_i[2])
                sin_i = math.sin(node_i[2])
                dx_world = node_j[0] - node_i[0]
                dy_world = node_j[1] - node_i[1]

                pred_dx = cos_i * dx_world + sin_i * dy_world
                pred_dy = -sin_i * dx_world + cos_i * dy_world
                pred_dtheta = wrap_angle(node_j[2] - node_i[2])

                err_x = pred_dx - meas_dx
                err_y = pred_dy - meas_dy
                err_theta = wrap_angle(pred_dtheta - meas_dtheta)
                error = np.array([[err_x], [err_y], [err_theta]])

                if is_loop:
                    norm_error = float(np.linalg.norm(error))
                    weight = 1.0 / (
                        1.0 + (norm_error / loop_cauchy_delta) ** 2
                    )
                    info_matrix = np.diag([150.0, 150.0, 400.0]) * weight
                else:
                    info_matrix = np.diag([100.0, 100.0, 200.0])

                Ji = np.array([
                    [-cos_i, -sin_i, pred_dy],
                    [sin_i, -cos_i, -pred_dx],
                    [0.0, 0.0, -1.0],
                ])

                Jj = np.array([
                    [cos_i, sin_i, 0.0],
                    [-sin_i, cos_i, 0.0],
                    [0.0, 0.0, 1.0],
                ])

                H_ii = Ji.T @ info_matrix @ Ji
                H_ij = Ji.T @ info_matrix @ Jj
                H_jj = Jj.T @ info_matrix @ Jj

                b_i = Ji.T @ info_matrix @ error
                b_j = Jj.T @ info_matrix @ error

                idx_i = 3 * i
                idx_j = 3 * j

                H[idx_i:idx_i + 3, idx_i:idx_i + 3] += H_ii
                H[idx_i:idx_i + 3, idx_j:idx_j + 3] += H_ij
                H[idx_j:idx_j + 3, idx_i:idx_i + 3] += H_ij.T
                H[idx_j:idx_j + 3, idx_j:idx_j + 3] += H_jj

                b[idx_i:idx_i + 3] += b_i.flatten()
                b[idx_j:idx_j + 3] += b_j.flatten()

            delta = spsolve(H.tocsc(), -b)
            if not np.all(np.isfinite(delta)):
                return None

            poses[:, 0] += delta[0::3]
            poses[:, 1] += delta[1::3]
            new_theta = poses[:, 2] + delta[2::3]
            poses[:, 2] = np.arctan2(np.sin(new_theta), np.cos(new_theta))

            if np.max(np.abs(delta)) < 1e-4:
                break

        return poses.tolist()

    def _apply_optimized_poses(self, snapshot, optimized):
        """Apply the result and carry its correction onto newer keyframes."""
        num_snapshot_poses = len(snapshot)

        if len(optimized) != num_snapshot_poses:
            self.get_logger().error(
                'Optimizer returned a different number of poses than its snapshot.'
            )
            return False

        if len(self.poses) < num_snapshot_poses:
            self.get_logger().error(
                'Live pose graph is shorter than the optimizer snapshot.'
            )
            return False

        # Replace the poses that were included in the worker snapshot.
        for pose, (x, y, theta) in zip(
            self.poses[:num_snapshot_poses], optimized
        ):
            pose.x = x
            pose.y = y
            pose.theta = wrap_angle(theta)

        # Keyframes appended during optimization inherit the same rigid correction.
        old_x, old_y, old_theta = snapshot[-1]
        new_x, new_y, new_theta = optimized[-1]

        dtheta = wrap_angle(new_theta - old_theta)
        c = math.cos(dtheta)
        s = math.sin(dtheta)
        dx = new_x - (c * old_x - s * old_y)
        dy = new_y - (s * old_x + c * old_y)

        for pose in self.poses[num_snapshot_poses:]:
            x, y = pose.x, pose.y
            pose.x = dx + c * x - s * y
            pose.y = dy + s * x + c * y
            pose.theta = wrap_angle(pose.theta + dtheta)

        return True

    def _poll_optimizer(self):
        """Collect and commit results on the ROS executor thread."""
        future = self._optimization_future
        if future is None or not future.done():
            return

        snapshot = self._optimization_snapshot
        try:
            optimized = future.result()
        except Exception as exc:
            self.get_logger().error(f'Graph optimization failed: {exc}')
            optimized = None

        self._optimization_future = None
        self._optimization_snapshot = None

        if optimized is None:
            self.get_logger().warning(
                'Graph optimization returned no valid result.'
            )
        elif self._apply_optimized_poses(snapshot, optimized):
            self._invalidate_submap_cache()
            self._update_map_to_odom()
            self.publish_visualization()
            self.get_logger().info(
                f'Graph optimized: updated {len(optimized)} keyframe poses.'
            )

        if self._optimization_pending:
            self._optimization_pending = False
            self.request_graph_optimization()

    def _update_map_to_odom(self):
        """Recompute map -> odom from the latest keyframe: T_m2o = T_map * T_odom^-1"""
        if not self.poses:
            return

        latest_node = self.poses[-1]
        theta_m2o = wrap_angle(latest_node.theta - latest_node.odom_theta)
        c = math.cos(theta_m2o)
        s = math.sin(theta_m2o)
        x_m2o = latest_node.x - (c * latest_node.odom_x - s * latest_node.odom_y)
        y_m2o = latest_node.y - (s * latest_node.odom_x + c * latest_node.odom_y)
        self.m2o = (x_m2o, y_m2o, theta_m2o)

    def publish_map_to_odom_tf(self):
        """Broadcast the dynamic map -> odom transform (timer driven, post-dated like slam_toolbox)"""
        if not self.poses:
            return

        x_m2o, y_m2o, theta_m2o = self.m2o

        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() + Duration(seconds=self.tf_postdate)).to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = x_m2o
        t.transform.translation.y = y_m2o
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta_m2o / 2.0)
        t.transform.rotation.w = math.cos(theta_m2o / 2.0)

        self.tf_broadcaster.sendTransform(t)

    def publish_visualization(self):
        if not self.poses:
            return

        now = self.get_clock().now().to_msg()

        # Trajectory Path
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = self.map_frame
        for pose in self.poses:
            p = PoseStamped()
            p.header.frame_id = self.map_frame
            p.pose.position.x = pose.x
            p.pose.position.y = pose.y
            path_msg.poses.append(p)
        self.path_pub.publish(path_msg)

        # Keyframe Pose Arrows (x, y, orientation)
        pose_array = PoseArray()
        pose_array.header.stamp = now
        pose_array.header.frame_id = self.map_frame
        for pose in self.poses:
            p = GeoPose()
            p.position.x = pose.x
            p.position.y = pose.y
            p.orientation.z = math.sin(pose.theta / 2.0)
            p.orientation.w = math.cos(pose.theta / 2.0)
            pose_array.poses.append(p)
        self.poses_pub.publish(pose_array)

        # Markers: Node Dots (Blue) & Loop Closure Lines (Red)
        markers = MarkerArray()

        # Node Dots
        node_dots = Marker()
        node_dots.header.frame_id = self.map_frame
        node_dots.ns = 'nodes'
        node_dots.id = 0
        node_dots.type = Marker.SPHERE_LIST
        node_dots.scale.x = 0.1
        node_dots.scale.y = 0.1
        node_dots.scale.z = 0.1
        node_dots.color.b = 1.0  # Blue
        node_dots.color.a = 1.0


        # Loop Closure Line (Red)
        loop_lines = Marker()
        loop_lines.header.frame_id = self.map_frame
        loop_lines.ns = 'loops'
        loop_lines.id = 1
        loop_lines.type = Marker.LINE_LIST
        loop_lines.scale.x = 0.05
        loop_lines.color.r = 1.0
        loop_lines.color.a = 1.0

        for c in self.constraints:
            if c.is_loop:
                loop_lines.points.append(Point(x=self.poses[c.from_id].x, y=self.poses[c.from_id].y, z=0.0))
                loop_lines.points.append(Point(x=self.poses[c.to_id].x, y=self.poses[c.to_id].y, z=0.0))

        for pose in self.poses:
            node_dots.points.append(Point(x=pose.x, y=pose.y, z=0.0))

        markers.markers.append(node_dots)
        markers.markers.append(loop_lines)
        self.graph_pub.publish(markers)

    # ------------------------------------------------------------------
    # CHANGED: input handling. Scans are paired with the odometry pose
    # interpolated at the scan's own timestamp (what slam_toolbox does via
    # its tf message filter), instead of "whatever scan arrived last".
    # ------------------------------------------------------------------
    def interpolate_odom(self, t):
        buf = self.odom_buffer
        if len(buf) < 2 or t < buf[0][0] or t > buf[-1][0]:
            return None
        for k in range(len(buf) - 1, 0, -1):
            if buf[k - 1][0] <= t:
                t0, x0, y0, th0 = buf[k - 1]
                t1, x1, y1, th1 = buf[k]
                a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                return (
                    x0 + a * (x1 - x0),
                    y0 + a * (y1 - y0),
                    wrap_angle(th0 + a * wrap_angle(th1 - th0)),
                )
        return None

    def odom_callback(self, msg: Odometry):
        """Buffer odometry, then process every queued scan that odometry now covers"""
        self.current_angular_vel = msg.twist.twist.angular.z
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if abs(self.current_angular_vel) > self.max_angular_velocity:
            self.last_high_rotation_time = stamp_sec

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_buffer.append((stamp_sec, x, y, theta))
        self.process_pending_scans()

    def process_pending_scans(self):
        if len(self.odom_buffer) < 2:
            return
        newest = self.odom_buffer[-1][0]
        oldest = self.odom_buffer[0][0]

        while self.scan_queue:
            t_scan, msg, pts = self.scan_queue[0]

            if t_scan > newest:
                if t_scan - newest > 1.0:
                    self.scan_queue.popleft()
                    self.get_logger().warn(
                        f'Scan is {t_scan - newest:.2f}s newer than odometry: clock offset between /scan and /odom? '
                        '(check chrony / Create3 time sync). Dropping scan.',
                        throttle_duration_sec=5.0)
                    continue
                break  # odometry has not caught up yet; wait for the next odom message

            self.scan_queue.popleft()
            if t_scan < oldest:
                self.get_logger().warn(
                    f'Scan is {oldest - t_scan:.2f}s older than buffered odometry: clock offset or large latency. Dropping scan.',
                    throttle_duration_sec=5.0)
                continue

            pose = self.interpolate_odom(t_scan)
            if pose is None:
                continue
            self.process_scan(t_scan, pose, msg, pts)

    def radius_outlier_removal(self, points):
        pts = np.asarray(points, dtype=float)
        if len(pts) == 0:
            return []

        neighbor_counts = cKDTree(pts).query_ball_point(
            pts, r=self.ror_radius, return_length=True
        )
        keep = neighbor_counts >= self.ror_min_neighbors + 1
        return pts[keep].tolist()

    def laser_callback(self, msg: LaserScan):
        """Converts raw LaserScan polar ranges to local 2D Cartesian (x, y) points, applies Radius Outlier Removal (ROR) and voxel downsampling"""
        points = []
        cos_yaw = math.cos(self.laser_yaw)
        sin_yaw = math.sin(self.laser_yaw)

        for i, r in enumerate(msg.ranges):
            # Skip invalid range readings
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue

            # Convert the laser-frame point into base_link coordinates.
            angle = msg.angle_min + i * msg.angle_increment
            x_lidar = r * math.cos(angle)
            y_lidar = r * math.sin(angle)
            x_base = self.laser_x + cos_yaw * x_lidar - sin_yaw * y_lidar
            y_base = self.laser_y + sin_yaw * x_lidar + cos_yaw * y_lidar
            points.append((x_base, y_base))

        # Radius Outlier Removal to remove inconsistent points
        points = self.radius_outlier_removal(points)

        # Voxel downsample to reduce point count for faster ICP
        pts = self.voxel_downsample(points)

        # Reference time of the scan: the lidar stamps the START of the sweep,
        # the robot pose that best explains the data is the middle of it.
        t_ref = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.use_scan_mid_time and msg.scan_time > 0.0:
            t_ref += 0.5 * msg.scan_time

        if self.require_time_sync:
            self.scan_queue.append((t_ref, msg, pts))
            self.process_pending_scans()
        elif self.odom_buffer:
            # Old behaviour for A/B testing: latest odom sample, whatever its stamp.
            _, ox, oy, oth = self.odom_buffer[-1]
            self.process_scan(t_ref, (ox, oy, oth), msg, pts)

    def process_scan(self, stamp_sec, odom_pose, scan_msg, scan_pts):
        """Keyframe creation / front-end ICP / loop closure for one time-aligned (scan, odom pose) pair"""
        if len(scan_pts) < 15:
            return

        time_since_rotation = stamp_sec - self.last_high_rotation_time
        if abs(self.current_angular_vel) > self.max_angular_velocity and time_since_rotation < self.settle_time:
            self.get_logger().warn(
                f'Angular velocity {self.current_angular_vel:.2f} rad/s exceeds max {self.max_angular_velocity:.2f} rad/s. '
                f'Skipping odometry update to avoid corrupting the pose graph.',
                throttle_duration_sec=5.0)
            return
        
        x, y, theta = odom_pose

        # Initialize First Node (Node 0)
        if not self.poses:
            first_node = Pose(
                node_id=0,
                stamp_sec=stamp_sec,
                map_x=x,
                map_y=y,
                map_theta=theta,
                odom_x=x,
                odom_y=y,
                odom_theta=theta,
                scan_points=scan_pts
            )
            self.poses.append(first_node)
            self.keyframe_scan_pub.publish(scan_msg)
            self.get_logger().info(f'Created initial pose node at odom ({x:.2f}, {y:.2f}, {theta:.2f})')
            self._update_map_to_odom()
            self.publish_visualization()
            return

        last_node = self.poses[-1]
        # Compute relative transformation from last node to current odometry

        global_dx = x - last_node.odom_x
        global_dy = y - last_node.odom_y
        dtheta = wrap_angle(theta - last_node.odom_theta)
        dist = math.hypot(global_dx, global_dy)

        if dist >= self.min_dist or abs(dtheta) >= self.min_angle:
            # Transform global delta to local frame of last node
            cos_odom = math.cos(last_node.odom_theta)
            sin_odom = math.sin(last_node.odom_theta)
            local_dx = cos_odom * global_dx + sin_odom * global_dy
            local_dy = -sin_odom * global_dx + cos_odom * global_dy

            # Front-End ICP Alignment: Align current scan against preceding submap
            submap_target = self.get_submap(len(self.poses) - 1, window_size=3)
            icp_dx, icp_dy, icp_dtheta, fitness, inlier_ratio = self.icp(
                scan_pts, submap_target, init_guess=(local_dx, local_dy, dtheta)
            )

            # CHANGED: accept the ICP correction only if the fit is tight AND the
            # correction is small relative to what odometry predicted.
            corr_d = math.hypot(icp_dx - local_dx, icp_dy - local_dy)
            corr_a = abs(wrap_angle(icp_dtheta - dtheta))
            accepted = (
                fitness < self.frontend_fitness_threshold
                and inlier_ratio >= self.frontend_min_inlier_ratio
                and corr_d < self.frontend_max_corr_dist
                and corr_a < self.frontend_max_corr_angle
            )
            if accepted:
                local_dx, local_dy, dtheta = icp_dx, icp_dy, icp_dtheta

            # Project new pose into current map frame
            cos_map = math.cos(last_node.theta)
            sin_map = math.sin(last_node.theta)
            map_x = last_node.x + cos_map * local_dx - sin_map * local_dy
            map_y = last_node.y + sin_map * local_dx + cos_map * local_dy
            map_theta = wrap_angle(last_node.theta + dtheta)

            # Update self.accum_dist
            self.accum_dist.append(self.accum_dist[-1] + dist)

            # Append new Keyframe Pose
            new_id = len(self.poses)
            new_node = Pose(
                node_id=new_id,
                stamp_sec=stamp_sec,
                map_x=map_x,
                map_y=map_y,
                map_theta=map_theta,
                odom_x=x,
                odom_y=y,
                odom_theta=theta,
                scan_points=scan_pts
            )
            self.poses.append(new_node)

            # Create Odometry Constraint (Edge) between last node and new node
            from_id = new_id - 1
            to_id = new_id
            odom_constraint = Constraint(
                from_id=from_id,
                to_id=to_id,
                dx=local_dx,
                dy=local_dy,
                dtheta=dtheta,
                is_loop=False
            )
            self.constraints.append(odom_constraint)
            self.get_logger().info(
                f"Added Keyframe Node {new_id} | Total Edges: {len(self.constraints)} | "
                f"ICP fit={fitness:.3f} inliers={inlier_ratio:.2f} corr={corr_d:.3f}m/{corr_a:.3f}rad "
                f"{'accepted' if accepted else 'REJECTED (using odometry)'}"
            )

            # Check for Loop Closure
            self.check_loop_closure()

            self._update_map_to_odom()
            self.keyframe_scan_pub.publish(scan_msg)

            # CHANGED: poses/markers only change when a keyframe is added or the
            # graph is optimized, so publish them here instead of at odom rate.
            self.publish_visualization()

    def save_trajectory_tum(self, filename="slam_trajectory_real.txt"):
        with open(filename, "w") as f:
            for pose in self.poses:
                qz = math.sin(pose.theta / 2.0)
                qw = math.cos(pose.theta / 2.0)
                f.write(f"{pose.stamp:.6f} {pose.x:.4f} {pose.y:.4f} 0.0000 0.0000 0.0000 {qz:.4f} {qw:.4f}\n")
        self.get_logger().info(f"Saved trajectory to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.do_save_trajectory_tum:
            node.save_trajectory_tum()
        node._optimizer_pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
