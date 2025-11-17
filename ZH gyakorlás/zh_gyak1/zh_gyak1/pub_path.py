import rclpy
from rclpy.node import Node
import math
import copy

from nav_msgs.msg import Path
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


class PublishPathNode(Node):

    def __init__(self):
        super().__init__("pub_path")
        self.get_logger().info("PublishPathNode has been started.")

        # változók
        all_positions = []
        self.transformed_positions_ = []

        # paraméterek
        file_path = self.declare_parameter("file_path", "/valami.txt")
        file_path = self.get_parameter("file_path").value

        self.target_frame_ = self.declare_parameter("frame_id", "map")
        self.target_frame_ = self.get_parameter("frame_id").value

        # File beolvasása
        with open(file_path, "r") as file:
            all_lines = file.readlines()

        positions = []
        for line in all_lines:
            position = line.strip().split("\t")
            x = float(position[0])
            y = float(position[1])
            z = float(position[2])
            yaw = float(position[3])
            positions.append([x, y, z, yaw])

        # Transzformáció a paraméterként bekért frame-be:
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer, self)

        # Transzformáció vizsgálata
        if tfBuffer.can_transform(
            self.target_frame_,
            "map",
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=0.1),
        ):
            # Transzformációs Mátrix létrehozása
            trans_to_target_frame = tfBuffer.lookup_transform(
                self.target_frame_,
                "map",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1),
            )

            # Transzformáció az egyes pontokra:
            for position in positions:
                pose = Pose()
                pose.position.x = position[0]
                pose.position.y = position[1]
                pose.position.z = position[2]
                quaternions = quaternion_from_euler(0.0, 0.0, math.radians(position[3]))
                pose.orientation.x = quaternions[0]
                pose.orientation.y = quaternions[1]
                pose.orientation.z = quaternions[2]
                pose.orientation.w = quaternions[3]

                self.transformed_positions_.append(
                    do_transform_pose(pose, trans_to_target_frame)
                )

        # Path message létrehozása
        # A file-ból kiolvasott és transzformált adatok kiküldése
        self.path_message_ = Path()
        self.path_message_.header.stamp = self.get_clock().now().to_msg()
        self.path_message_.header.frame_id = self.target_frame_

        for transformed_pose in self.transformed_positions_:
            transformed_posestamped = PoseStamped()
            transformed_posestamped.header.stamp = self.get_clock().now().to_msg()
            transformed_posestamped.header.frame_id = self.target_frame_

            transformed_posestamped.pose.position.x = transformed_pose.position.x
            transformed_posestamped.pose.position.x = transformed_pose.position.x
            transformed_posestamped.pose.position.y = transformed_pose.position.y
            transformed_posestamped.pose.position.z = transformed_pose.position.z
            transformed_posestamped.pose.orientation.x = transformed_pose.orientation.x
            transformed_posestamped.pose.orientation.y = transformed_pose.orientation.y
            transformed_posestamped.pose.orientation.z = transformed_pose.orientation.z
            transformed_posestamped.pose.orientation.w = transformed_pose.orientation.w

            self.path_message_.poses.append(transformed_posestamped)

        # MarkerArray message létrehozása
        min_x = 1000.0
        min_y = 1000.0
        max_x = -1000.0
        max_y = -1000.0

        for one_pose in self.path_message_.poses:
            max_x = max(max_x, one_pose.pose.position.x)
            max_y = max(max_y, one_pose.pose.position.y)
            min_x = min(min_x, one_pose.pose.position.x)
            min_y = min(min_y, one_pose.pose.position.y)

        self.markerarray_message_ = MarkerArray()
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.target_frame_
        marker.ns = "sphere"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.scale.x = 0.1  # 10 cm-es legyen a gömb
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()

        # Először a minimum pontba rakjuk ki
        marker.pose.position.x = min_x
        marker.pose.position.y = min_y
        marker.pose.position.z = 0.0

        self.markerarray_message_.markers.append(copy.deepcopy(marker))

        # Majd a maximum pontba
        marker.id = 1
        marker.pose.position.x = max_x
        marker.pose.position.y = max_y

        self.markerarray_message_.markers.append(copy.deepcopy(marker))

        # Publisher létrehozása
        self.path_pub_ = self.create_publisher(Path, "/path", 1)
        self.marker_pub_ = self.create_publisher(MarkerArray, "/viz", 1)

        # Timer létrehozása az adatküldéshez
        timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):

        # Path adatok kiküldése
        self.path_pub_.publish(self.path_message_)
        self.get_logger().info("Path data has been published from PathPublishNode.")

        # MarkerArray adatok kiküldése
        self.marker_pub_.publish(self.markerarray_message_)
        self.get_logger().info("MarkerArray data has been published from PathPubNode.")


def main(args=None):
    rclpy.init(args=args)
    node = PublishPathNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
