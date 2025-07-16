#!/usr/bin/env python3
"""
ROS 2 node that detects 4×4 ArUco tags in the left RGB image published by
the ZED 2i wrapper and outputs `vision_msgs/Detection3DArray`.

Topic flow (default):
  SUB  /zed2i/left/image_rect_color   sensor_msgs/Image
  SUB  /zed2i/left/camera_info        sensor_msgs/CameraInfo
  PUB  /aruco_detections              vision_msgs/Detection3DArray
"""

import cv2
import numpy as np
from cv2 import aruco
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, \
                            ObjectHypothesisWithPose

class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # --------------- parameters ---------------- #
        self.declare_parameter("marker_size", 0.20)  # metres
        self.declare_parameter("rgb_topic",
                               "/zed2i/left/image_rect_color")
        self.declare_parameter("cam_info_topic",
                               "/zed2i/left/camera_info")
        p = self.get_parameter
        self.marker_size = float(p("marker_size").value)
        rgb_topic  = p("rgb_topic").value
        info_topic = p("cam_info_topic").value

        # --------------- OpenCV objects ------------ #
        self.bridge = CvBridge()
        self.dict   = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.params = aruco.DetectorParameters()

        # --------------- ROS I/O ------------------- #
        self.cam_info_msg = None
        self.create_subscription(CameraInfo, info_topic,
                                 self.info_cb, 1)
        self.create_subscription(Image, rgb_topic,
                                 self.img_cb,  10)
        self.pub = self.create_publisher(Detection3DArray,
                                         "aruco_detections", 10)

        self.get_logger().info("ArucoDetector ready 😊")

    # ------------------------------------------------ #
    def info_cb(self, msg: CameraInfo):
        self.cam_info_msg = msg

    # ------------------------------------------------ #
    def img_cb(self, msg: Image):
        if self.cam_info_msg is None:
            return  # wait until intrinsics received

        img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray    = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.dict, parameters=self.params)

        det_arr            = Detection3DArray()
        det_arr.header     = msg.header
        det_arr.detections = []

        if ids is not None:
            # Camera intrinsics
            K     = np.array(self.cam_info_msg.k).reshape(3, 3)
            dist  = np.array(self.cam_info_msg.d)
            size_mm = self.marker_size * 1000.0  # OpenCV wants millimetres

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, size_mm, K, dist)

            for i, tvec in enumerate(tvecs):
                det = Detection3D()
                # OpenCV coordinate frame: +x right, +y down, +z out of cam
                # Convert to ROS REP‑103 (x forward, y left, z up) by swapping axes
                x_c, y_c, z_c = tvec[0] / 1000.0  # metres
                det.bbox.center.position.x =  x_c
                det.bbox.center.position.y = -y_c
                det.bbox.center.position.z =  z_c

                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = str(int(ids[i][0]))
                hypo.hypothesis.score    = 1.0
                det.results.append(hypo)
                det_arr.detections.append(det)

        self.pub.publish(det_arr)

# -------------------- main ------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
