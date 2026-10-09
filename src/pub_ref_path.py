from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node


class PubRefPath(Node):
    def __init__(self):
        super().__init__('pub_ref_path')
        self.publisher_ = self.create_publisher(Path, '/reference_path', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/vehicle_pose', 10)
        self.create_timer(1.0, self.publish_ref_path)
        self.create_timer(0.1, self.publish_vehicle_pose)

    def publish_vehicle_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        self.pose_publisher_.publish(pose)

    def publish_ref_path(self):
        path = Path()
        path.header.frame_id = 'map'
        for i in range(300):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 0.2 * i
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.publisher_.publish(path)


def main():
    rclpy.init()
    node = PubRefPath()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
