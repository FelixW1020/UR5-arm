"""ROS 2 node: run an Ultralytics YOLO model on camera images.

Publishes
  ~/detections   vision_msgs/Detection2DArray  (header copied from the image)
  ~/debug_image  sensor_msgs/Image             (boxes drawn; only when subscribed)

This node knows nothing about depth, TF or the robot. Swap in any detector that
publishes the same message and the rest of the stack keeps working.
"""
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        # Any Ultralytics model: yolov8n.pt, yolo11n.pt, or your own fine-tuned best.pt.
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('classes', 'cup')       # comma-separated COCO names; '' = all
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('iou', 0.5)
        self.declare_parameter('device', 'auto')       # 'auto', 'cpu', 'cuda:0'
        self.declare_parameter('image_size', 640)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._model = YOLO(g('model'))
        names = self._model.names  # {id: name}
        wanted = [c.strip() for c in g('classes').split(',') if c.strip()]
        unknown = [c for c in wanted if c not in names.values()]
        if unknown:
            raise ValueError(f'Model {g("model")} has no classes {unknown}. '
                             f'Available: {sorted(names.values())}')
        self._class_ids = [i for i, n in names.items() if n in wanted] or None
        self._conf = g('confidence')
        self._iou = g('iou')
        self._device = None if g('device') in ('', 'auto') else g('device')
        self._imgsz = g('image_size')

        self._bridge = CvBridge()
        self._det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        # depth=1 + best effort: always run on the newest frame, drop the backlog.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, g('image_topic'), self._on_image, qos)
        self.get_logger().info(
            f'YOLO model {g("model")} loaded, classes={wanted or "all"}, conf>={self._conf}')

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self._model.predict(
            frame, conf=self._conf, iou=self._iou, classes=self._class_ids,
            device=self._device, imgsz=self._imgsz, verbose=False)[0]

        out = Detection2DArray()
        out.header = msg.header
        names = result.names
        for xywh, conf, cls in zip(result.boxes.xywh.tolist(), result.boxes.conf.tolist(),
                                   result.boxes.cls.tolist()):
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = float(xywh[0])
            det.bbox.center.position.y = float(xywh[1])
            det.bbox.size_x = float(xywh[2])
            det.bbox.size_y = float(xywh[3])
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = names[int(cls)]
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            out.detections.append(det)
        self._det_pub.publish(out)

        if self._debug_pub.get_subscription_count() > 0:
            dbg = self._bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
            dbg.header = msg.header
            self._debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
