#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class EKFSLAM(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')
        self.map_frame = 'map'
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        # SLAM states (mu_t, Sigma_t, N_t)
        self.N = 0
        self.mu = np.zeros((3, 1))
        self.Sigma = np.zeros((3, 3))

        # Noise parameters
        # NOTE: process noise is now MOTION-PROPORTIONAL (Thrun-style alpha
        # parameters), not a fixed rate per second. Real odometry error (wheel
        # slip, encoder quantization) scales with how far/fast the robot
        # actually moved and turned, not with wall-clock time. A fixed-per-
        # second model stays too small during fast motion/sharp turns, making
        # the filter overconfident (Sigma looks fine, but the true error has
        # already exceeded it) -- which causes correct landmark re-associations
        # to fail the Mahalanobis gate and get registered as duplicate new
        # landmarks instead of correcting the pose. See predict_step().
        self.alpha1 = 0.08   # position std contribution per meter translated
        self.alpha2 = 0.05   # position std contribution per radian rotated
        self.alpha3 = 0.35   # heading std contribution per radian rotated
        self.alpha4 = 0.03   # heading std contribution per meter translated
        self.min_pos_std_rate = 0.01   # m/s floor -- noise even while "still" (slip, quantization)
        self.min_rot_std_rate = 0.01   # rad/s floor
        self.Q_t = np.diag([0.2, 0.1]) ** 2          # Measurement noise covariance
        
        # Outlier rejection thresholds
        self.alpha = 9.21           # Mahalanobis Distance threshold (~99% confidence)
        self.euclidean_gate_base = 1.5   # Meters: minimum gate radius, used as a cheap
                                          # pre-filter to skip far-away landmarks before
                                          # doing the (more expensive, more correct)
                                          # Mahalanobis test. This is NOT the final
                                          # association decision -- it just widens
                                          # automatically as pose uncertainty grows so it
                                          # never blocks a re-association that the
                                          # Mahalanobis test would otherwise accept.
        self.euclidean_gate_sigma_mult = 3.0

        # Publishers & Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, '/ekf_slam/path', 10)
        self.landmarks_pub = self.create_publisher(MarkerArray, '/ekf_slam/map_landmarks', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.landmark_sub = self.create_subscription(MarkerArray, '/landmark_markers', self.landmark_callback, 10)

        # State tracking for synchronized loop
        self.latest_v = 0.0
        self.latest_w = 0.0
        self.latest_measurements = []
        self.latest_odom_stamp = None
        self.latest_odom_x = 0.0
        self.latest_odom_y = 0.0
        self.latest_odom_theta = 0.0
        self.last_timer_time = None
        self._diag_counter = 0
        self._diag_last_N = 0
        self._diag_last_x = 0.0
        self._diag_last_y = 0.0

        # Synchronized SLAM Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.slam_loop)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg: Odometry):
        # Only store the latest odometry values
        self.latest_v = msg.twist.twist.linear.x
        self.latest_w = msg.twist.twist.angular.z
        
        # Save pose for TF broadcasing
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.latest_odom_theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.latest_odom_x = msg.pose.pose.position.x
        self.latest_odom_y = msg.pose.pose.position.y
        self.latest_odom_stamp = msg.header.stamp

    def landmark_callback(self, msg: MarkerArray):
        # Process markers into a list of measurements and save
        measurements = []
        for marker in msg.markers:
            local_x = marker.pose.position.x
            local_y = marker.pose.position.y
            r = math.hypot(local_x, local_y)
            if r < 0.3:
                continue
            phi = self.normalize_angle(math.atan2(local_y, local_x))
            measurements.append(np.array([[r], [phi]]))
        self.latest_measurements = measurements

    def slam_loop(self):
        # Synchronized processing loop
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_timer_time is None:
            self.last_timer_time = current_time
            return
            
        dt = current_time - self.last_timer_time
        self.last_timer_time = current_time
        
        # 1. Prediction Step
        if dt > 0:
            self.predict_step(self.latest_v, self.latest_w, dt)
        
        # 2. Correction Step
        update_start = self.get_clock().now().nanoseconds / 1e9
        if self.latest_measurements:
            self.update_step(self.latest_measurements)
            self.latest_measurements = [] # Clear after processing to avoid duplicate updates
        update_elapsed_ms = (self.get_clock().now().nanoseconds / 1e9 - update_start) * 1000.0

        # Diagnostics: once every ~5s (50 ticks @ 10Hz), log pose uncertainty,
        # landmark count, actual dt, and update_step runtime. If dt starts
        # exceeding 0.1s or update_elapsed_ms climbs as N grows, the timer is
        # falling behind and predict_step is integrating over larger, noisier
        # steps than the R_t tuning assumes -- a sign the map is too big for
        # this per-step full-Sigma EKF formulation to keep up with in real time.
        self._diag_counter += 1
        if self._diag_counter % 50 == 0:
            pos_std = math.sqrt(max(self.Sigma[0, 0], self.Sigma[1, 1]))
            new_landmarks_since_last = self.N - self._diag_last_N
            cur_x, cur_y = float(self.mu[0, 0]), float(self.mu[1, 0])
            dist_since_last = math.hypot(cur_x - self._diag_last_x, cur_y - self._diag_last_y)
            landmarks_per_meter = (new_landmarks_since_last / dist_since_last) if dist_since_last > 1e-3 else float('inf')
            self._diag_last_N = self.N
            self._diag_last_x, self._diag_last_y = cur_x, cur_y

            self.get_logger().info(
                f"[diag] N={self.N} (+{new_landmarks_since_last}) pos_std={pos_std:.3f}m "
                f"dt={dt*1000:.1f}ms update_step={update_elapsed_ms:.1f}ms "
                f"dist_moved={dist_since_last:.2f}m"
            )
            # Only warn if landmarks are appearing much faster than the pose is
            # covering new ground (e.g. many new landmarks while barely moving,
            # or repeatedly re-driving the same area and still gaining
            # landmarks). Covering genuinely new ground while N grows is
            # expected and NOT flagged.
            if new_landmarks_since_last > 2 and landmarks_per_meter > 3.0:
                self.get_logger().warn(
                    "[diag] Landmark count is climbing much faster than the pose is "
                    "covering new ground -- this suggests already-mapped landmarks "
                    "are being re-registered as new ones (data association failure) "
                    "rather than genuinely new features being explored."
                )
            
        # 3. Publish Visuals
        if self.latest_odom_stamp is not None:
            self.publish_tf_and_visuals(self.latest_odom_stamp, self.latest_odom_x, self.latest_odom_y, self.latest_odom_theta)

    def predict_step(self, v, w, dt):
        mu_x, mu_y, mu_theta = self.mu[0, 0], self.mu[1, 0], self.mu[2, 0]
        state_dim = 3 + 2 * self.N
        G_t = np.eye(state_dim)

        if abs(w) > 1e-4:
            new_mu_x = mu_x - (v / w) * math.sin(mu_theta) + (v / w) * math.sin(mu_theta + w * dt)
            new_mu_y = mu_y + (v / w) * math.cos(mu_theta) - (v / w) * math.cos(mu_theta + w * dt)
            new_mu_theta = self.normalize_angle(mu_theta + w * dt)
            
            G_t[0, 2] =  (v / w) * math.cos(mu_theta) - (v / w) * math.cos(mu_theta + w * dt)
            G_t[1, 2] =  (v / w) * math.sin(mu_theta) - (v / w) * math.sin(mu_theta + w * dt)
        else:
            new_mu_x = mu_x + v * dt * math.cos(mu_theta)
            new_mu_y = mu_y + v * dt * math.sin(mu_theta)
            new_mu_theta = mu_theta
            
            G_t[0, 2] = -v * dt * math.sin(mu_theta)
            G_t[1, 2] =  v * dt * math.cos(mu_theta)

        self.mu[0, 0] = new_mu_x
        self.mu[1, 0] = new_mu_y
        self.mu[2, 0] = new_mu_theta

        # Motion-proportional process noise: scale with actual distance
        # translated and angle rotated THIS tick, not with dt alone. A robot
        # that sat still should get ~0 injected noise; one that just did a
        # fast, sharp turn should get a lot. This keeps Sigma an honest
        # reflection of true error even during aggressive motion, instead of
        # letting the filter become overconfident.
        trans = abs(v) * dt
        rot = abs(w) * dt

        pos_std = self.alpha1 * trans + self.alpha2 * rot + self.min_pos_std_rate * dt
        rot_std = self.alpha3 * rot + self.alpha4 * trans + self.min_rot_std_rate * dt
        R_t_effective = np.diag([pos_std, pos_std, rot_std]) ** 2

        F_x = np.hstack((np.eye(3), np.zeros((3, 2 * self.N))))
        self.Sigma = G_t @ self.Sigma @ G_t.T + F_x.T @ R_t_effective @ F_x

    def update_step(self, measurements):
        for z_i in measurements:
            r_i, phi_i = z_i[0, 0], z_i[1, 0]
            mu_x, mu_y, mu_theta = self.mu[0, 0], self.mu[1, 0], self.mu[2, 0]

            # Calculate expected global location of this measurement
            mu_N_x = mu_x + r_i * math.cos(phi_i + mu_theta)
            mu_N_y = mu_y + r_i * math.sin(phi_i + mu_theta)

            # Dynamic Euclidean pre-gate: widen the cheap distance check as pose
            # uncertainty grows, so it never rejects a re-association that the
            # Mahalanobis test below would otherwise accept. A fixed-radius gate
            # here would increasingly reject valid re-observations of already-mapped
            # landmarks the longer the robot runs without a correction -- which
            # blocks the very mechanism (re-association) that's supposed to bound
            # drift, and turns ordinary dead-reckoning drift into runaway drift.
            pose_std = math.sqrt(max(self.Sigma[0, 0], self.Sigma[1, 1]))
            dynamic_gate = self.euclidean_gate_base + self.euclidean_gate_sigma_mult * pose_std

            min_pi = float('inf')
            best_k = -1
            best_H = None
            best_Psi = None
            best_z_hat = None

            for k in range(1, self.N + 2):
                if k == self.N + 1:
                    pi_k = self.alpha
                    if pi_k < min_pi:
                        min_pi = pi_k
                        best_k = k
                    continue

                m_x = self.mu[3 + 2 * (k - 1), 0]
                m_y = self.mu[3 + 2 * (k - 1) + 1, 0]
                
                # EUCLIDEAN GATING
                # Skip Mahalanobis calculation if the feature is physically too far away
                dist = math.hypot(m_x - mu_N_x, m_y - mu_N_y)
                if dist > dynamic_gate:
                    continue

                delta_x = m_x - mu_x
                delta_y = m_y - mu_y
                q_k = delta_x**2 + delta_y**2
                sqrt_q = math.sqrt(q_k)

                if q_k < 1e-6:
                    continue

                z_hat = np.array([
                    [sqrt_q],
                    [self.normalize_angle(math.atan2(delta_y, delta_x) - mu_theta)]
                ])

                state_dim = 3 + 2 * self.N
                F_xk = np.zeros((5, state_dim))
                F_xk[0:3, 0:3] = np.eye(3)
                F_xk[3, 3 + 2 * (k - 1)] = 1.0
                F_xk[4, 3 + 2 * (k - 1) + 1] = 1.0

                H_low = (1.0 / q_k) * np.array([
                    [-sqrt_q * delta_x, -sqrt_q * delta_y, 0.0,  sqrt_q * delta_x,  sqrt_q * delta_y],
                    [          delta_y,          -delta_x, -q_k,         -delta_y,           delta_x]
                ])
                H_k = H_low @ F_xk

                Psi_k = H_k @ self.Sigma @ H_k.T + self.Q_t
                dz = z_i - z_hat
                dz[1, 0] = self.normalize_angle(dz[1, 0])
                
                try:
                    Psi_inv = np.linalg.inv(Psi_k)
                    pi_k = (dz.T @ Psi_inv @ dz)[0, 0]
                except np.linalg.LinAlgError:
                    continue

                if pi_k < min_pi:
                    min_pi = pi_k
                    best_k = k
                    best_H = H_k
                    best_Psi = Psi_k
                    best_z_hat = z_hat

            if best_k == self.N + 1:
                # Initialize new landmark
                self.N += 1
                self.mu = np.vstack((self.mu, [[mu_N_x], [mu_N_y]]))
                
                # MATHEMATICALLY ACCURATE COVARIANCE INITIALIZATION
                # Compute Jacobians to map pose uncertainty to the new landmark
                theta = mu_theta + phi_i
                c = math.cos(theta)
                s = math.sin(theta)
                
                J_R = np.array([
                    [1.0, 0.0, -r_i * s],
                    [0.0, 1.0,  r_i * c]
                ])
                
                J_Z = np.array([
                    [c, -r_i * s],
                    [s,  r_i * c]
                ])
                
                # Covariance of new landmark with all existing states
                cov_lx = J_R @ self.Sigma[:3, :] 
                # Internal 2x2 covariance of the new landmark
                cov_ll = J_R @ self.Sigma[:3, :3] @ J_R.T + J_Z @ self.Q_t @ J_Z.T 
                
                # Expand global Sigma matrix safely
                self.Sigma = np.vstack((self.Sigma, cov_lx))
                self.Sigma = np.hstack((self.Sigma, np.vstack((cov_lx.T, cov_ll))))
                
                continue  

            if best_k != -1 and best_H is not None:
                try:
                    Psi_inv = np.linalg.inv(best_Psi)
                    K_i = self.Sigma @ best_H.T @ Psi_inv
                except np.linalg.LinAlgError:
                    continue

                dz = z_i - best_z_hat
                dz[1, 0] = self.normalize_angle(dz[1, 0])
                
                self.mu += K_i @ dz
                self.mu[2, 0] = self.normalize_angle(self.mu[2, 0])
                self.Sigma = (np.eye(self.Sigma.shape[0]) - K_i @ best_H) @ self.Sigma

    def publish_tf_and_visuals(self, stamp, odom_x, odom_y, odom_theta):
        map_x, map_y, map_theta = float(self.mu[0, 0]), float(self.mu[1, 0]), float(self.mu[2, 0])

        tf_theta = self.normalize_angle(map_theta - odom_theta)
        tf_x = map_x - (odom_x * math.cos(tf_theta) - odom_y * math.sin(tf_theta))
        tf_y = map_y - (odom_x * math.sin(tf_theta) + odom_y * math.cos(tf_theta))

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = tf_x
        t.transform.translation.y = tf_y
        t.transform.rotation.z = math.sin(tf_theta / 2.0)
        t.transform.rotation.w = math.cos(tf_theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.orientation.z = math.sin(map_theta / 2.0)
        pose.pose.orientation.w = math.cos(map_theta / 2.0)
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        marker_array = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for k in range(1, self.N + 1):
            mx = float(self.mu[3 + 2 * (k - 1), 0])
            my = float(self.mu[3 + 2 * (k - 1) + 1, 0])

            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = "map_landmarks"
            marker.id = k
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = mx
            marker.pose.position.y = my
            marker.pose.position.z = 0.2
            marker.scale.x, marker.scale.y, marker.scale.z = 0.25, 0.25, 0.4
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
            marker_array.markers.append(marker)

        self.landmarks_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()