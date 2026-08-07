#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Wheel parameters
        self.R = 0.0325 # Wheel Radius in meters
        self.L = 0.175 # Distance between wheels in meters
        self.alpha = 0.95 # Complementary filter alpha

        # Robot pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.imu_omega = 0.0
        self.omega = 0.0

        self.last_time = self.get_clock().now() # Last time stamp for odometry calculation

        # Subscriptions and publishers
        self.joint_sub = self.create_subscription( # JointState data subscription
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.odom_pub = self.create_publisher( # Odometry data publisher
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) # TF broadcaster
        self.get_logger().info("Odometry initialized.")

    def imu_callback(self, msg: Imu):
        self.imu_omega = msg.angular_velocity.z

    def joint_callback(self, msg: JointState):
        # Guard against empty velocity arrays
        if not msg.velocity or len(msg.velocity) < 2:
            return

        # Attempt dynamic index lookup by joint name (handles TurtleBot4 / Create3 joint names)
        left_idx, right_idx = None, None
        for i, name in enumerate(msg.name):
            if 'left' in name and 'wheel' in name:
                left_idx = i
            elif 'right' in name and 'wheel' in name:
                right_idx = i

        # Retrieve velocities using matched joint indices or fallback to first two array elements
        try:
            if left_idx is not None and right_idx is not None:
                v_l = msg.velocity[left_idx] * self.R
                v_r = msg.velocity[right_idx] * self.R
            else:
                v_l = msg.velocity[0] * self.R
                v_r = msg.velocity[1] * self.R
        except IndexError:
            return

        v = (v_r + v_l) / 2.0 
        encoder_omega = (v_r - v_l) / self.L 
    
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 
        self.last_time = current_time

        if dt <= 0:
            return

        theta_imu = self.theta + self.imu_omega * dt
        theta_encoder = self.theta + encoder_omega * dt

        self.theta = self.alpha * theta_imu + (1 - self.alpha) * theta_encoder
        self.omega = self.alpha * self.imu_omega + (1 - self.alpha) * encoder_omega
        
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta)) 

        self.publish_odom_msg(v, self.omega) 
        self.publish_tf()

    def publish_odom_msg(self, v, omega):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation (quaternion)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Set velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set rotation (quaternion)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()