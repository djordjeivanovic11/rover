#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import onnxruntime as ort

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        # parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('classes_file', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/zed2i/left/image_rect_color')
        self.declare_parameter('output_topic', '/detected_objects')

        model_path = self.get_parameter('model_path').value
        classes_file = self.get_parameter('classes_file').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        img_topic = self.get_parameter('image_topic').value
        out_topic = self.get_parameter('output_topic').value

        # load class names
        with open(classes_file, 'r') as f:
            self.classes = [l.strip() for l in f if l.strip()]

        # initialize ONNX runtime
        self.session = ort.InferenceSession(model_path)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, img_topic, self.image_callback, 5)
        self.pub = self.create_publisher(
            Detection2DArray, out_topic, 5)

    def image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # --- run your pre/post processing here ---
        # For example:
        #   input_tensor = preprocess(cv_img)
        #   outputs = self.session.run(None, {'input': input_tensor})
        #   detections = postprocess(outputs, self.conf_thresh, self.classes)
        #
        # Here we’ll just publish an empty array:
        dets = Detection2DArray()
        dets.header = msg.header
        # fill dets.detections with your Detection2D messages...
        self.pub.publish(dets)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    rclpy.shutdown()
