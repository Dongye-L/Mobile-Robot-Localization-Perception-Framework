import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class UwbPoseAdapter(Node):
    def __init__(self):
        super().__init__('uwb_pose_adapter')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/uwb_pose_cov', 10
        )
        self.subscription = self.create_subscription(
            PoseStamped, '/uwb_pose', self.callback, 10
        )

    def callback(self, source):
        msg = PoseWithCovarianceStamped()
        msg.header = source.header
        msg.pose.pose = source.pose

        # 初始假设 UWB 位置标准差约 0.3 m；未使用的姿态/Z 协方差设大。
        covariance = [0.0] * 36
        covariance[0] = 0.09
        covariance[7] = 0.09
        covariance[14] = 1e6
        covariance[21] = 1e6
        covariance[28] = 1e6
        covariance[35] = 1e6
        msg.pose.covariance = covariance
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()