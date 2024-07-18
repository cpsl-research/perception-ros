import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2 as LidarMsg
from vision_msgs.msg import Detection3DArray

from avstack_bridge.sensors import LidarSensorBridge
from avstack_bridge.detections import DetectionBridge

from avstack.modules.perception.object3d import MMDetObjectDetector3D


class LidarPerception(Node):
    def __init__(self):
        super().__init__("perception")       
        self.model = MMDetObjectDetector3D(model='pointpillars', dataset='carla-vehicle')

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

        # publish lidar detections
        self.publisher_pc = self.create_publisher(
            Detection3DArray,
            "detections_3d",
            qos_profile=qos,
        )

    def pc_callback(self, pc_msg: LidarMsg) -> Detection3DArray:
        pc_avstack = LidarSensorBridge.pc2_to_avstack(pc_msg)
        dets_avstack = self.model(pc_avstack)
        dets_ros = DetectionBridge.avstack_to_detections_3d(dets_avstack, header=pc_msg.header)
        self.publisher_pc.publish(dets_ros)


def main(args=None):
    rclpy.init(args=args)

    percep = LidarPerception()

    rclpy.spin(percep)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    percep.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
