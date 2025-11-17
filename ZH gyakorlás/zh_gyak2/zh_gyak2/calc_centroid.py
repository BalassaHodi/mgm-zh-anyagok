import rclpy
from rclpy.node import Node

import math

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pcl2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped


class CalculateCentroidNode(Node):

    def __init__(self):
        super().__init__("calc_centroid")
        self.get_logger().info("CalculateCentroidNode has been started.")

        # transformation
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # subscriptions
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 1)

        # publishers
        self.pc_pub = self.create_publisher(PointCloud2, "/cloud", 1)
        self.marker_pub = self.create_publisher(Marker, "/centroid", 1)
        self.pos_pub = self.create_publisher(PoseStamped, "/tfed_centroid", 1)

    def scan_cb(self, msg: LaserScan):
        # Create xyz
        points = []
        for i in range(len(msg.ranges)):
            if msg.range_min < msg.ranges[i] < msg.range_max:
                x = msg.ranges[i] * math.cos(msg.angle_min + msg.angle_increment * i)
                y = msg.ranges[i] * math.sin(msg.angle_min + msg.angle_increment * i)
                z = msg.ranges[i]
                points.append([x, y, z])

        point_cloud = pcl2.create_cloud_xyz32(msg.header, points)
        self.pc_pub.publish(point_cloud)

        # centroid
        sum = [0, 0, 0]
        for point in points:
            sum[0] += point[0]
            sum[1] += point[1]
            sum[2] += point[2]
        centroid = [sum[0] / len(points), sum[1] / len(points), sum[2] / len(points)]

        # create marker message
        marker = Marker()
        marker.header = msg.header
        marker.ns = "scan"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        # transformation
        centroid_posestamped = PoseStamped()
        centroid_posestamped.header = msg.header
        centroid_posestamped.pose.position.x = centroid[0]
        centroid_posestamped.pose.position.y = centroid[1]
        centroid_posestamped.pose.position.z = centroid[2]

        if self.tfBuffer.can_transform(
            "map",
            msg.header.frame_id,
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=0.1),
        ):
            trans2map = self.tfBuffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time()
            )
            tfed_centroid = PoseStamped()
            tfed_centroid = do_transform_pose_stamped(centroid_posestamped, trans2map)
            self.pos_pub.publish(tfed_centroid)
            self.get_logger().info("Transformed centroid has been sent.")


def main(args=None):
    rclpy.init(args=args)
    node = CalculateCentroidNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
