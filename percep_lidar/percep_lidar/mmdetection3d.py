import rclpy
from avstack.modules.perception.object3d import MMDetObjectDetector3D
from avstack_bridge import Bridge
from avstack_bridge.detections import DetectionBridge
from avstack_bridge.sensors import LidarSensorBridge
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 as LidarMsg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray


class LidarPerception(Node):
    def __init__(self, verbose: bool = False):
        super().__init__("perception")
        self.declare_parameter("perception_model", "pointpillars")
        self.declare_parameter("perception_dataset", "carla-vehicle")

        # initialize models
        param_model = self.get_parameter("perception_model").value
        param_dataset = self.get_parameter("perception_dataset").value
        self.model = MMDetObjectDetector3D(
            model=param_model,
            dataset=param_dataset,
        )
        self.get_logger().info(
            f"Initialized {param_model} model on {param_dataset} dataset"
        )
        self.verbose = verbose

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )

        # listen to transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos)

        # subscribe to point cloud in the same namespace
        self.subscriber_pc = self.create_subscription(
            LidarMsg, "point_cloud", self.pc_callback, qos_profile=qos
        )

        # publish lidar detections
        self.publisher_dets = self.create_publisher(
            Detection3DArray,
            "detections_3d",
            qos_profile=qos,
        )

    def pc_callback(self, pc_msg: LidarMsg) -> Detection3DArray:
        """Run perception model when we receive lidar data

        We need to obtain the actual reference frame for the lidar sensor
        because the perception model runs detection on a nominal sensor
        height. Therefore, we need to adjust the perception outcomes
        based on the difference between the nominal height and the realized
        sensor height.
        """
        # get the transformations
        # HACK: assume the sensor is e.g., agent0/lidar0
        try:
            frame_agent = pc_msg.header.frame_id.split("/")[0]
            tf_world_agent = self.tf_buffer.lookup_transform(
                "world",
                frame_agent,
                pc_msg.header.stamp,
            )
            tf_agent_lidar = self.tf_buffer.lookup_transform(
                frame_agent,
                pc_msg.header.frame_id,
                pc_msg.header.stamp,
            )
            ref_agent_world = Bridge.tf2_to_reference(tf_world_agent)
            ref_lidar_agent = Bridge.tf2_to_reference(
                tf_agent_lidar, reference=ref_agent_world
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform point cloud for perception")
            return

        # run the pipeline
        pc_avstack = LidarSensorBridge.pc2_to_avstack(pc_msg)
        pc_avstack.calibration.reference = ref_lidar_agent
        dets_avstack = self.model(pc_avstack)
        if self.verbose:
            self.get_logger().info(
                f"Lidar perception generated {len(dets_avstack)} detections"
            )
        dets_ros = DetectionBridge.avstack_to_detectionarray(
            dets_avstack, header=pc_msg.header
        )
        self.publisher_dets.publish(dets_ros)


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
