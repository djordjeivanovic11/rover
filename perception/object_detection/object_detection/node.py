#!/usr/bin/env python3
"""
URC Object Detection Node
========================

Detects mission-critical objects for University Rover Challenge:
- Science samples (rocks, minerals, biological specimens)
- Equipment (bottles, toolboxes, containers, cables)
- Navigation markers (flags, posts, signs)
- Obstacles (large rocks, equipment, structures)

Uses YOLO model with ONNX runtime for efficient inference.
Publishes Detection2DArray for downstream pose estimation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import onnxruntime as ort
import cv2
import numpy as np
from typing import List, Tuple, Optional

class URCObjectDetection(Node):
    def __init__(self):
        super().__init__('urc_object_detection')
        
        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('classes_file', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('image_topic', '/zed2i/left/image_rect_color')
        self.declare_parameter('output_topic', '/detected_objects')
        self.declare_parameter('max_detections', 100)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        classes_file = self.get_parameter('classes_file').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.iou_thresh = self.get_parameter('iou_threshold').value
        self.input_size = self.get_parameter('input_size').value
        self.max_detections = self.get_parameter('max_detections').value
        img_topic = self.get_parameter('image_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Load URC mission classes
        self.classes = self._load_urc_classes(classes_file)
        self.get_logger().info(f"Loaded {len(self.classes)} URC object classes")

        # Initialize ONNX runtime with optimization
        # Try GPU first, fallback to CPU for Jetson compatibility
        try:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.session = ort.InferenceSession(model_path, providers=providers)
            self.get_logger().info("ONNX Runtime initialized with GPU support")
        except Exception as e:
            self.get_logger().warn(f"GPU provider failed: {e}, using CPU only")
            providers = ['CPUExecutionProvider']
            self.session = ort.InferenceSession(model_path, providers=providers)
            self.get_logger().info("ONNX Runtime initialized with CPU only")
        
        # Get model input/output info
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]
        
        self.get_logger().info(f"Model input: {self.input_name}")
        self.get_logger().info(f"Model outputs: {self.output_names}")

        # ROS2 setup
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, img_topic, self.image_callback, 5)
        self.pub = self.create_publisher(Detection2DArray, out_topic, 5)
        
        # Performance monitoring
        self.frame_count = 0
        self.create_timer(5.0, self.log_performance)
        
        self.get_logger().info("URC Object Detection Node initialized")

    def _load_urc_classes(self, classes_file: str) -> List[str]:
        """Load URC-specific object classes"""
        try:
            with open(classes_file, 'r') as f:
                classes = [line.strip() for line in f if line.strip()]
            return classes
        except Exception as e:
            self.get_logger().error(f"Failed to load classes: {e}")
            # Fallback URC classes
            return [
                'rock', 'mineral', 'biological_sample', 'bottle', 'toolbox', 
                'container', 'cable', 'flag', 'post', 'sign', 'obstacle'
            ]

    def preprocess_image(self, image: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        """
        Preprocess image for YOLO inference
        Returns: (preprocessed_tensor, scale_factor, (pad_x, pad_y))
        """
        original_shape = image.shape[:2]  # (height, width)
        
        # Resize with aspect ratio preservation (letterboxing)
        scale = min(self.input_size / original_shape[0], self.input_size / original_shape[1])
        new_shape = (int(original_shape[1] * scale), int(original_shape[0] * scale))
        
        # Resize image
        resized = cv2.resize(image, new_shape, interpolation=cv2.INTER_LINEAR)
        
        # Create padded image
        pad_x = (self.input_size - new_shape[0]) // 2
        pad_y = (self.input_size - new_shape[1]) // 2
        
        padded = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        padded[pad_y:pad_y + new_shape[1], pad_x:pad_x + new_shape[0]] = resized
        
        # Convert to tensor format: HWC -> CHW, BGR -> RGB, normalize
        tensor = padded.transpose(2, 0, 1).astype(np.float32) / 255.0
        tensor = np.expand_dims(tensor, axis=0)  # Add batch dimension
        
        return tensor, scale, (pad_x, pad_y)

    def postprocess_detections(self, outputs: List[np.ndarray], scale: float, 
                             pads: Tuple[int, int], original_shape: Tuple[int, int]) -> List[dict]:
        """
        Postprocess YOLO outputs to get final detections
        """
        # Assuming YOLO format: [batch, num_detections, 85] where 85 = 4(bbox) + 1(conf) + 80(classes)
        predictions = outputs[0][0]  # Remove batch dimension
        
        # Filter by confidence
        conf_mask = predictions[:, 4] >= self.conf_thresh
        predictions = predictions[conf_mask]
        
        if len(predictions) == 0:
            return []
        
        # Extract boxes, confidences, and class predictions
        boxes = predictions[:, :4]  # x_center, y_center, width, height (normalized)
        confidences = predictions[:, 4]
        class_scores = predictions[:, 5:]
        
        # Get class with highest score for each detection
        class_ids = np.argmax(class_scores, axis=1)
        class_confidences = np.max(class_scores, axis=1)
        
        # Final confidence = objectness * class_confidence
        final_confidences = confidences * class_confidences
        
        # Convert to absolute coordinates and remove padding
        pad_x, pad_y = pads
        boxes[:, 0] = (boxes[:, 0] * self.input_size - pad_x) / scale  # x_center
        boxes[:, 1] = (boxes[:, 1] * self.input_size - pad_y) / scale  # y_center
        boxes[:, 2] = boxes[:, 2] * self.input_size / scale  # width
        boxes[:, 3] = boxes[:, 3] * self.input_size / scale  # height
        
        # Convert to x1, y1, x2, y2 format for NMS
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2
        
        # Apply Non-Maximum Suppression
        nms_boxes = np.column_stack([x1, y1, x2, y2])
        keep_indices = self.nms(nms_boxes, final_confidences, self.iou_thresh)
        
        # Build final detections
        detections = []
        for idx in keep_indices[:self.max_detections]:
            if class_ids[idx] < len(self.classes):
                detection = {
                    'bbox': [float(x1[idx]), float(y1[idx]), float(x2[idx]), float(y2[idx])],
                    'confidence': float(final_confidences[idx]),
                    'class_id': int(class_ids[idx]),
                    'class_name': self.classes[class_ids[idx]]
                }
                detections.append(detection)
        
        return detections

    def nms(self, boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> List[int]:
        """Non-Maximum Suppression implementation"""
        if len(boxes) == 0:
            return []
        
        # Sort by confidence scores
        indices = np.argsort(scores)[::-1]
        keep = []
        
        while len(indices) > 0:
            current = indices[0]
            keep.append(current)
            
            if len(indices) == 1:
                break
                
            # Calculate IoU with remaining boxes
            current_box = boxes[current]
            remaining_boxes = boxes[indices[1:]]
            
            # Calculate intersection
            x1 = np.maximum(current_box[0], remaining_boxes[:, 0])
            y1 = np.maximum(current_box[1], remaining_boxes[:, 1])
            x2 = np.minimum(current_box[2], remaining_boxes[:, 2])
            y2 = np.minimum(current_box[3], remaining_boxes[:, 3])
            
            intersection = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
            
            # Calculate union
            current_area = (current_box[2] - current_box[0]) * (current_box[3] - current_box[1])
            remaining_areas = (remaining_boxes[:, 2] - remaining_boxes[:, 0]) * \
                            (remaining_boxes[:, 3] - remaining_boxes[:, 1])
            union = current_area + remaining_areas - intersection
            
            # Calculate IoU
            iou = intersection / (union + 1e-6)
            
            # Keep boxes with IoU below threshold
            indices = indices[1:][iou <= iou_threshold]
        
        return keep

    def create_detection_msg(self, detections: List[dict], header) -> Detection2DArray:
        """Convert detections to ROS2 Detection2DArray message"""
        msg = Detection2DArray()
        msg.header = header
        
        for det in detections:
            detection = Detection2D()
            
            # Bounding box
            x1, y1, x2, y2 = det['bbox']
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            
            # Object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = det['confidence']
            
            detection.results.append(hypothesis)
            msg.detections.append(detection)
        
        return msg

    def image_callback(self, msg: Image):
        """Main inference callback"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            original_shape = cv_image.shape[:2]
            
            # Preprocess
            input_tensor, scale, pads = self.preprocess_image(cv_image)
            
            # Run inference
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
            
            # Postprocess
            detections = self.postprocess_detections(outputs, scale, pads, original_shape)
            
            # Create and publish ROS message
            detection_msg = self.create_detection_msg(detections, msg.header)
            self.pub.publish(detection_msg)
            
            # Log detections
            if detections:
                classes_detected = [det['class_name'] for det in detections]
                self.get_logger().debug(f"Detected: {classes_detected}")
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Detection failed: {e}")

    def log_performance(self):
        """Log performance metrics"""
        fps = self.frame_count / 5.0
        self.get_logger().info(f"Detection FPS: {fps:.2f}")
        self.frame_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = URCObjectDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
