#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Wheel parameters
        self.R = 0.0325 # Wheel Radius in meters
        self.L = 0.175 # Distance between wheels in meters

        # Robot pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now() # Last time stamp for odometry calculation

        # Subscriptions and publishers
        self.joint_sub = self.create_subscription( # JointState data subscription
            JointState,
            '/joint_states_raw',
            self.joint_callback,
            10
        )

        self.odom_pub = self.create_publisher( # Odometry data publisher
            Odometry,
            'robot1/odom',
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) # TF broadcaster
        self.get_logger().info("Odometry initialized.")

    def joint_callback(self, msg):
        v_r = msg.velocity[0] * self.R # Right wheel velocity
        v_l = msg.velocity[1] * self.R # Left wheel velocity

        # Calculate linear and angular velocity
        v = (v_r + v_l) / 2.0 # Linear velocity
        omega = (v_r - v_l) / self.L # Angular velocity

        # Time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 # Time difference in seconds
        self.last_time = current_time

        # Update robot pose using differential drive kinematics
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta)) # Keep theta within [-pi, pi]

        self.publish_odom_msg(v, omega) # Publish odometry message

        self.publish_tf() # Publish transform for TF (odom -> base_link)

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