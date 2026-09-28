"""Visualization utilities for the SRU navigation controller (ROS1 port)."""

import rospy
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from sru_nav_go2 import constants


def _stamp_from_sec(sec):
    """Build a rospy.Time stamp from a float seconds value."""
    return rospy.Time.from_sec(float(sec))


class VisualizationManager:
    """Manages visualization markers for the navigation controller."""

    def __init__(self, node=None):
        # `node` retained for API parity with the original class. Not used in ROS1.
        self.node = node

    def publish_twist_marker(self, twist_msg, robot_odom_time, robot_frame_id, publisher):
        if robot_frame_id is None:
            return

        marker = Marker()
        marker.header = Header()
        marker.header.stamp = _stamp_from_sec(robot_odom_time)
        marker.header.frame_id = robot_frame_id

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = constants.TWIST_MARKER_ID

        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(
            x=twist_msg.linear.x * constants.TWIST_MARKER_SCALE,
            y=twist_msg.linear.y * constants.TWIST_MARKER_SCALE,
            z=0.0,
        )
        marker.points = [start_point, end_point]

        marker.scale = Vector3(x=0.2, y=0.4, z=0.4)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        marker.frame_locked = False

        publisher.publish(marker)

    def publish_target_vector_marker(self, target_vec_b, robot_odom_time, robot_frame_id, publisher):
        if robot_frame_id is None:
            return

        marker = Marker()
        marker.header = Header()
        marker.header.stamp = _stamp_from_sec(robot_odom_time)
        marker.header.frame_id = robot_frame_id

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = constants.TARGET_VECTOR_MARKER_ID

        vec = target_vec_b.flatten()

        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(x=float(vec[0]), y=float(vec[1]), z=float(vec[2]))
        marker.points = [start_point, end_point]

        marker.scale = Vector3(x=0.1, y=0.2, z=0.2)
        marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.frame_locked = False

        publisher.publish(marker)
