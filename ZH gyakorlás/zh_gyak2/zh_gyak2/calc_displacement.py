import rclpy
from rclpy.node import Node

import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64


class CalculateDisplacementNode(Node):

    def __init__(self):
        super().__init__("calc_displacement")
        self.get_logger().info("CalculateDisplacementNode has been started.")

        # Variables
        self.previous_pose = Pose()
        self.previous_pose.position.x = 0.0
        self.previous_pose.position.y = 0.0

        self.sum_displacement = Float64()
        self.sum_displacement.data = 0.0

        self.previous_sum_disp = 0.0

        # subscription
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 1)

        # publishers
        self.disp_pub = self.create_publisher(Float64, "/elmozdulas", 1)
        self.pos_pub = self.create_publisher(Odometry, "/poz", 1)

    def odom_cb(self, msg: Odometry):
        actual_pose = Pose()
        actual_pose.position.x = msg.pose.pose.position.x
        actual_pose.position.y = msg.pose.pose.position.y

        dx = actual_pose.position.x - self.previous_pose.position.x
        dy = actual_pose.position.y - self.previous_pose.position.y

        displacement = math.sqrt(dx**2 + dy**2)

        self.previous_pose.position.x = actual_pose.position.x
        self.previous_pose.position.y = actual_pose.position.y

        self.sum_displacement.data += displacement

        self.disp_pub.publish(self.sum_displacement)
        # self.get_logger().info("Float64 data has been sent.")

        if self.sum_displacement.data - self.previous_sum_disp >= 1.0:
            self.pos_pub.publish(msg)
            self.get_logger().info("Odometry data has been sent.")
            self.previous_sum_disp = self.sum_displacement.data


def main(args=None):
    rclpy.init(args=args)
    node = CalculateDisplacementNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
