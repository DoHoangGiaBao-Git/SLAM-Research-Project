from math import sqrt
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

from robot_model.robot_feedback_node import RobotFeedbackNode


def test_odometry_callback_corrects_heading_without_rotating_position():
    node = object.__new__(RobotFeedbackNode)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=999)))
    transforms = []
    node.transform_broadcaster = SimpleNamespace(
        sendTransform=transforms.append)

    msg = Odometry()
    msg.header.stamp = Time(sec=431)
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 3.0
    msg.pose.pose.orientation.w = 1.0

    node.odometry_callback(msg)

    transform = transforms[0]
    assert transform.header.stamp == Time(sec=999)
    assert transform.header.frame_id == 'odom'
    assert transform.child_frame_id == 'base_link'
    assert transform.transform.translation.x == 1.0
    assert transform.transform.translation.y == 2.0
    assert transform.transform.translation.z == 3.0
    assert transform.transform.rotation.x == 0.0
    assert transform.transform.rotation.y == 0.0
    assert abs(transform.transform.rotation.z - sqrt(0.5)) < 1e-12
    assert abs(transform.transform.rotation.w - sqrt(0.5)) < 1e-12
