from math import atan2, cos, sin, tau

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster


TICKS_PER_REVOLUTION = 1975.0


class RobotFeedbackNode(Node):

    def __init__(self):
        super().__init__('robot_feedback_node')
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.joint_positions = {
            'revolute_1': 0.0,
            'revolute_2': 0.0,
        }

        self.create_subscription(
            Int32, 'encoder/left',
            lambda msg: self.encoder_callback('revolute_1', msg), 10)
        self.create_subscription(
            Int32, 'encoder/right',
            lambda msg: self.encoder_callback('revolute_2', msg), 10)
        self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

    def encoder_callback(self, joint_name, msg):
        self.joint_positions[joint_name] = (
            msg.data * tau / TICKS_PER_REVOLUTION)

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_positions)
        joint_state.position = list(self.joint_positions.values())
        self.joint_state_publisher.publish(joint_state)

    def odometry_callback(self, msg):
        transform = TransformStamped()
        # The microcontroller odometry stamp is not in the host ROS clock domain.
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = msg.header.frame_id
        transform.child_frame_id = msg.child_frame_id

        position = msg.pose.pose.position
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z

        orientation = msg.pose.pose.orientation
        yaw = atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        ) + tau / 4.0
        transform.transform.rotation.z = sin(yaw / 2.0)
        transform.transform.rotation.w = cos(yaw / 2.0)

        self.transform_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = RobotFeedbackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
