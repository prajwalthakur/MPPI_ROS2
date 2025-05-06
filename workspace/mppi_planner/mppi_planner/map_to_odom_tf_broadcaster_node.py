#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time

class MapToOdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('map_to_odom_tf_broadcaster_node')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Changed topic name to /odom
            self.odom_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
            """
            Callback function to handle new Odometry messages and broadcast the
            transform from map to base_link (as per your requirement).
            """
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'base_footprint'
            t.child_frame_id = 'map'  # Changed to base_link as per your requirement

            # Create a Vector3 message for the translation
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z

            t.transform.rotation = msg.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()