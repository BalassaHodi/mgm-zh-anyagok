import rclpy
import math
from rclpy.node import Node

from nav_msgs.msg import Odometry
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion


class SavePathNode(Node):

    def __init__(self):
        super().__init__("save_path")
        self.get_logger().info("SavePathNode has been started.")

        # változók
        self.previous_x_ = 0.0
        self.previous_y_ = 0.0
        self.previous_z_ = 0.0

        # paraméter definiálása
        topic_name = self.declare_parameter("topic_name", "/odom")
        topic_name = self.get_parameter("topic_name").value

        self.file_path_ = self.declare_parameter("file_path", "/valami.txt")
        self.file_path_ = self.get_parameter("file_path").value

        self.distance_ = self.declare_parameter("distance", 0.1)
        self.distance_ = self.get_parameter("distance").value

        # subscriber létrehozása
        self.sub_ = self.create_subscription(Odometry, topic_name, self.sub_cb, 1)

        # a tranformációhoz szükséges változók
        self.tfBuffer_ = tf2_ros.Buffer()
        self.tfListener_ = tf2_ros.TransformListener(self.tfBuffer_, self)

        # File-t üressé tesszük
        with open(self.file_path_, "w") as file:
            file.write("")

    def sub_cb(self, message: Odometry):
        # Megvizsgáljuk az előző és mostani pontok közti távolságot
        # Mostani pontok
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        z = message.pose.pose.position.z

        # az egyes pontok távolsága
        dx = x - self.previous_x_
        dy = y - self.previous_y_
        dz = z - self.previous_z_

        # a két pozíció távolsága
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist >= self.distance_:
            actual_pose = Pose()

            # Megvizsgáljuk, hogy a mentendő adatok a map frame-ben vannak-e
            if message.header.frame_id == "map":
                actual_pose = message.pose.pose
            else:
                # Megvizsgáljuk, hogy lehet-e transzformálni
                if self.tfBuffer_.can_transform(
                    "map",
                    message.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1),
                ):
                    # Létrehozzuk a transzformációs mátrixot
                    trans2map = self.tfBuffer_.lookup_transform(
                        "map",
                        message.header.frame_id,
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=0.1),
                    )

                    # Létrehozzuk a transzformációt
                    actual_pose = do_transform_pose(message.pose.pose, trans2map)

            # Kiküldjük az adatokat a file-ba a megfelelő formátumban
            # pozíciókat méterben (abban vannak)
            actual_x = actual_pose.position.x
            actual_y = actual_pose.position.y
            actual_z = actual_pose.position.z

            # legyezési szöget fokban, ezt át kell váltani:
            quaternion = [
                actual_pose.orientation.x,
                actual_pose.orientation.y,
                actual_pose.orientation.z,
                actual_pose.orientation.w,
            ]

            # csak a z irányú elfordulás kell (2D-ben az az elfordulás)
            actual_yaw = math.degrees(euler_from_quaternion(quaternion)[2])

            # File-ba írás
            with open(self.file_path_, "a") as file:
                file.write(
                    f"{actual_x:.2f}\t{actual_y:.2f}\t{actual_z:.2f}\t{actual_yaw:.2f}\n"
                )
                self.get_logger().info(
                    f"Data has been written to file: {self.file_path_}"
                )

            self.previous_x_ = x
            self.previous_y_ = y
            self.previous_z_ = z


def main(args=None):
    rclpy.init(args=args)
    node = SavePathNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
