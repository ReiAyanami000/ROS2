"""Validated device odometry to ROS Odometry and optional TF."""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdometryPublisher:
    def __init__(self, node, odom_frame, base_frame, publish_tf, pose_variance, twist_variance):
        self.node, self.odom_frame, self.base_frame = node, odom_frame, base_frame
        self.publisher = node.create_publisher(Odometry, '/odom', 5)
        self.tf = TransformBroadcaster(node) if publish_tf else None
        self.pose_variance, self.twist_variance = pose_variance, twist_variance

    def publish(self, values):
        msg = Odometry()
        # V0.1 uses reception/publication time. MCU uptime is never ROS wall time.
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(values[3])
        msg.pose.pose.position.y = float(values[4])
        msg.pose.pose.orientation.z = math.sin(values[5] / 2)
        msg.pose.pose.orientation.w = math.cos(values[5] / 2)
        msg.twist.twist.linear.x = float(values[6])
        msg.twist.twist.angular.z = float(values[7])
        for i in range(6):
            msg.pose.covariance[i * 7] = self.pose_variance[i]
            msg.twist.covariance[i * 7] = self.twist_variance[i]
        self.publisher.publish(msg)
        if self.tf:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(tf)
