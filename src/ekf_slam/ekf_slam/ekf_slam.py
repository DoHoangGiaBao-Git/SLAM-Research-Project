#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import geometry_msgs.msg as geometry_msgs
import math
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class EKFSLAMNode(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')
        
        # ----------------------------------------------------------------------
        # 0. Safe ROS 2 Parameter Setup
        # ----------------------------------------------------------------------
        # Check if the launch file already declared it to prevent crashing
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        
        # ----------------------------------------------------------------------
        # 1. EKF-SLAM Parameter Setup
        # ----------------------------------------------------------------------
        self.n_state = 3        # Robot base dimensions (x, y, theta)
        self.num_landmarks = 0  # Dynamic counter for active mapped landmarks
        
        # Initial State Vector
        self.mu = np.zeros((self.n_state, 1))  
        
        # Initial Robot Pose Covariance Matrix
        self.sigma = np.diag([1e-4, 1e-4, 1e-4])
        
        # Motion Noise Parameters in Control Space
        self.alpha = [0.05, 0.01, 0.05, 0.01] 
        
        # Sensor Measurement Noise Covariance Matrix
        self.std_r = 0.25               
        self.std_phi = np.radians(10.0)  
        self.Qt = np.diag([self.std_r**2, self.std_phi**2])
        
        # Chi-Square Data Association Threshold
        self.alpha_threshold = 20.0     
        
        # Tracking states
        self.last_time = None
        self.is_initialized = False

        # --- RAW ODOMETRY TRACKING ---
        self.raw_odom_x = 0.0
        self.raw_odom_y = 0.0
        self.raw_odom_theta = 0.0

        # ----------------------------------------------------------------------
        # 2. Global Frame Configurations & TF Broadcaster
        # ----------------------------------------------------------------------
        self.map_frame = 'map'
        self.odom_frame = 'robot1/odom'  
        self.tf_broadcaster = TransformBroadcaster(self)

        # ----------------------------------------------------------------------
        # 3. ROS 2 Subscriptions & Publishers
        # ----------------------------------------------------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            'robot1/odom',
            self.odom_callback,
            10
        )

        self.landmark_sub = self.create_subscription(
            Marker,
            'robot1/landmark_markers',
            self.landmark_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            geometry_msgs.PoseWithCovarianceStamped,
            'robot1/ekf_slam_pose',
            10
        )

        self.map_pub = self.create_publisher(
            Marker,
            '/ekf_slam/map',
            10
        )
        
        self.get_logger().info("2D EKF-SLAM Node initialized successfully.")

    def yaw_from_quaternion(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def broadcast_map_to_odom_tf(self, stamp):
        if not self.is_initialized:
            return

        map_to_base_x = float(self.mu[0, 0])
        map_to_base_y = float(self.mu[1, 0])
        map_to_base_theta = float(self.mu[2, 0])

        odom_to_base_x = self.raw_odom_x
        odom_to_base_y = self.raw_odom_y
        odom_to_base_theta = self.raw_odom_theta

        map_to_odom_theta = normalize_angle(map_to_base_theta - odom_to_base_theta)

        cos_t = math.cos(map_to_odom_theta)
        sin_t = math.sin(map_to_odom_theta)
        
        map_to_odom_x = map_to_base_x - (odom_to_base_x * cos_t - odom_to_base_y * sin_t)
        map_to_odom_y = map_to_base_y - (odom_to_base_x * sin_t + odom_to_base_y * cos_t)

        t = TransformStamped()
        t.header.stamp = stamp  
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = map_to_odom_x
        t.transform.translation.y = map_to_odom_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(map_to_odom_theta / 2.0)
        t.transform.rotation.w = math.cos(map_to_odom_theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        self.raw_odom_x = msg.pose.pose.position.x
        self.raw_odom_y = msg.pose.pose.position.y
        self.raw_odom_theta = self.yaw_from_quaternion(msg.pose.pose.orientation)

        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if not self.is_initialized:
            self.last_time = curr_time
            self.mu[0, 0] = self.raw_odom_x
            self.mu[1, 0] = self.raw_odom_y
            self.mu[2, 0] = self.raw_odom_theta
            self.is_initialized = True
            return

        dt = curr_time - self.last_time
        self.last_time = curr_time
        
        if dt <= 0.0:
            return

        vt = msg.twist.twist.linear.x
        wt = msg.twist.twist.angular.z
        theta_prev = self.mu[2, 0]

        if abs(wt) < 1e-5:
            dx = vt * math.cos(theta_prev) * dt
            dy = vt * math.sin(theta_prev) * dt
            dtheta = 0.0
        else:
            dx = -(vt / wt) * math.sin(theta_prev) + (vt / wt) * math.sin(theta_prev + wt * dt)
            dy = (vt / wt) * math.cos(theta_prev) - (vt / wt) * math.cos(theta_prev + wt * dt)
            dtheta = wt * dt

        self.mu[0, 0] += dx
        self.mu[1, 0] += dy
        self.mu[2, 0] = normalize_angle(self.mu[2, 0] + dtheta)

        g_t = np.eye(3)
        if abs(wt) < 1e-5:
            g_t[0, 2] = -vt * math.sin(theta_prev) * dt
            g_t[1, 2] =  vt * math.cos(theta_prev) * dt
        else:
            g_t[0, 2] = (vt / wt) * math.cos(theta_prev) - (vt / wt) * math.cos(theta_prev + wt * dt)
            g_t[1, 2] = (vt / wt) * math.sin(theta_prev) - (vt / wt) * math.sin(theta_prev + wt * dt)

        Rt = np.diag([
            self.alpha[0] * (vt**2) + self.alpha[1] * (wt**2),
            self.alpha[2] * (vt**2) + self.alpha[3] * (wt**2),
            0.01  
        ])

        dim = self.sigma.shape[0]
        self.sigma[0:3, 0:3] = g_t @ self.sigma[0:3, 0:3] @ g_t.T + Rt
        
        if dim > 3:
            self.sigma[0:3, 3:dim] = g_t @ self.sigma[0:3, 3:dim]
            self.sigma[3:dim, 0:3] = self.sigma[0:3, 3:dim].T

        self.broadcast_map_to_odom_tf(msg.header.stamp)
        self.publish_pose(msg.header.stamp)

    def landmark_callback(self, msg: Marker):
        if not self.is_initialized or len(msg.points) == 0:
            return

        for pt in msg.points:
            r_i = math.sqrt(pt.x**2 + pt.y**2)
            phi_i = normalize_angle(math.atan2(pt.y, pt.x))
            z_i = np.array([[r_i], [phi_i]])

            rx = self.mu[0, 0]
            ry = self.mu[1, 0]
            rtheta = self.mu[2, 0]

            mx_new = rx + r_i * math.cos(phi_i + rtheta)
            my_new = ry + r_i * math.sin(phi_i + rtheta)

            pi_scores = []

            for k in range(self.num_landmarks):
                idx = 3 + 2 * k
                delta_x = self.mu[idx, 0] - rx
                delta_y = self.mu[idx+1, 0] - ry
                
                dist_to_existing = math.sqrt((mx_new - self.mu[idx, 0])**2 + (my_new - self.mu[idx+1, 0])**2)
                
                if dist_to_existing > 0.8:
                    pi_scores.append(float('inf'))
                    continue
                
                q = delta_x**2 + delta_y**2
                sqrt_q = math.sqrt(q) if q > 1e-6 else 1e-3

                z_hat = np.array([
                    [sqrt_q],
                    [normalize_angle(math.atan2(delta_y, delta_x) - rtheta)]
                ])

                H_low = np.zeros((2, 5))
                H_low[0, 0], H_low[0, 1], H_low[0, 2] = -sqrt_q * delta_x, -sqrt_q * delta_y, 0.0
                H_low[1, 0], H_low[1, 1], H_low[1, 2] = delta_y, -delta_x, -q
                H_low[0, 3], H_low[0, 4] = sqrt_q * delta_x, sqrt_q * delta_y
                H_low[1, 3], H_low[1, 4] = -delta_y, delta_x
                H_low = H_low / (q if q > 1e-6 else 1e-3)

                H_k = np.zeros((2, self.mu.shape[0]))
                H_k[:, 0:3] = H_low[:, 0:3]
                H_k[:, idx:idx+2] = H_low[:, 3:5]

                Psi_k = H_k @ self.sigma @ H_k.T + self.Qt

                y = z_i - z_hat
                y[1, 0] = normalize_angle(y[1, 0])
                
                try:
                    pi_k = float(y.T @ np.linalg.inv(Psi_k) @ y)
                except np.linalg.LinAlgError:
                    pi_k = float('inf')
                    
                pi_scores.append(pi_k)

            pi_scores.append(self.alpha_threshold)
            j_selected = np.argmin(pi_scores)

            if j_selected == self.num_landmarks:
                self.mu = np.vstack((self.mu, [[mx_new], [my_new]]))
                n_old = self.sigma.shape[0]
                sigma_new = np.zeros((n_old + 2, n_old + 2))
                sigma_new[0:n_old, 0:n_old] = self.sigma
                sigma_new[n_old:n_old+2, n_old:n_old+2] = np.diag([0.2, 0.2])
                self.sigma = sigma_new
                self.num_landmarks += 1
            else:
                idx = 3 + 2 * j_selected
                delta_x = self.mu[idx, 0] - rx
                delta_y = self.mu[idx+1, 0] - ry
                q = delta_x**2 + delta_y**2
                sqrt_q = math.sqrt(q) if q > 1e-6 else 1e-3

                z_hat = np.array([
                    [sqrt_q],
                    [normalize_angle(math.atan2(delta_y, delta_x) - rtheta)]
                ])

                H_low = np.zeros((2, 5))
                H_low[0, 0], H_low[0, 1], H_low[0, 2] = -sqrt_q * delta_x, -sqrt_q * delta_y, 0.0
                H_low[1, 0], H_low[1, 1], H_low[1, 2] = delta_y, -delta_x, -q
                H_low[0, 3], H_low[0, 4] = sqrt_q * delta_x, sqrt_q * delta_y
                H_low[1, 3], H_low[1, 4] = -delta_y, delta_x
                H_low = H_low / (q if q > 1e-6 else 1e-3)

                H_j = np.zeros((2, self.mu.shape[0]))
                H_j[:, 0:3] = H_low[:, 0:3]
                H_j[:, idx:idx+2] = H_low[:, 3:5]

                Psi_j = H_j @ self.sigma @ H_j.T + self.Qt
                
                try:
                    K_i = self.sigma @ H_j.T @ np.linalg.inv(Psi_j)
                    y = z_i - z_hat
                    y[1, 0] = normalize_angle(y[1, 0])
                    
                    self.mu += K_i @ y
                    self.mu[2, 0] = normalize_angle(self.mu[2, 0])
                    
                    I = np.eye(self.sigma.shape[0])
                    self.sigma = (I - K_i @ H_j) @ self.sigma
                except np.linalg.LinAlgError:
                    continue

        self.broadcast_map_to_odom_tf(msg.header.stamp)
        self.publish_map(msg.header.stamp)

    def publish_pose(self, stamp):
        msg = geometry_msgs.PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        
        msg.pose.pose.position.x = float(self.mu[0, 0])
        msg.pose.pose.position.y = float(self.mu[1, 0])
        
        yaw = self.mu[2, 0]
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        cov = np.zeros((6, 6))
        cov[0, 0] = self.sigma[0, 0]
        cov[1, 1] = self.sigma[1, 1]
        cov[5, 5] = self.sigma[2, 2]
        msg.pose.covariance = cov.flatten().tolist()
        
        self.pose_pub.publish(msg)

    def publish_map(self, stamp):
        if self.num_landmarks == 0:
            return
            
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'ekf_slam_map'
        marker.id = 2
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        for i in range(self.num_landmarks):
            idx = 3 + 2 * i
            p = geometry_msgs.Point()
            p.x = float(self.mu[idx, 0])
            p.y = float(self.mu[idx+1, 0])
            p.z = 0.0
            marker.points.append(p)
            
        self.map_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()