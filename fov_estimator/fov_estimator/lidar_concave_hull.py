import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2 as LidarMsg
from avstack_bridge.sensors import LidarSensorBridge

from avstack.modules.perception.fov_estimator import ConcaveHullLidarFOVEstimator


class LidarFovEstimator(Node):
    def __init__(self):
        super().__init__("fov_estimator")
        self.model = ConcaveHullLidarFOVEstimator()

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        # subscribe to point cloud in the same namespace
        self.subscriber_pc = self.create_subscription(
            LidarMsg,
            "point_cloud",
            self.pc_callback,
            qos_profile=qos
        )

        # publish fov model as point cloud
        self.publisher_pc = self.create_publisher(
            LidarMsg,
            "fov",
            qos_profile=qos,
        )

    def pc_callback(self, pc_msg: LidarMsg) -> LidarMsg:
        pc_avstack = LidarSensorBridge.pc2_to_avstack(pc_msg)
        fov_avstack = self.model(pc_avstack)
        pc_ros = LidarSensorBridge.pc2_to_avstack(fov_avstack)
        self.publisher_fov.publish(pc_ros)


def main(args=None):
    rclpy.init(args=args)

    percep = LidarFovEstimator()

    rclpy.spin(percep)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    percep.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
