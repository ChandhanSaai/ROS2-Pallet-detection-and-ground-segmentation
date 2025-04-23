#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class GroundSegmentationNode(Node):
    def __init__(self):
        super().__init__('ground_segmentation')
        self.bridge = CvBridge()

        # Load your trained YOLOv11 segmentation model for ground
        # please replace the path with the actual path to your model
        self.model = YOLO('/home/chandhan/pallet_ws/src/pallet_segmentation/models/ground_segment/best.pt')

        # QoS to match bag settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to RGB image
        self.image_sub = self.create_subscription(
            Image,
            '/robot1/zed2i/left/image_rect_color', # Change topic name as needed
            self.image_callback,
            qos_profile)

        # Publisher for overlay
        self.image_pub = self.create_publisher(
            Image, '/inference/ground_segmentation', 10)

        self.get_logger().info("Ground Segmentation Node Started")
        
        # Load the model

    def image_callback(self, msg):
        self.get_logger().info("Received image")

        original_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


        # Resize image to 640x640 for YOLO inference
        resized_frame = cv2.resize(original_frame, (640, 640), interpolation=cv2.INTER_LINEAR)

        # Run inference on resized frame
        results = self.model.predict(source=resized_frame, conf=0.7, verbose=False)[0]

        # Initialize overlay mask
        fullsize_overlay = np.zeros_like(original_frame)

        if results.masks is not None:
            masks = results.masks.data.cpu().numpy()
            for mask in masks:
                # Resize mask to original frame size
                upscaled_mask = cv2.resize(mask.astype(np.uint8), (original_frame.shape[1], original_frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                fullsize_overlay[upscaled_mask.astype(bool)] = [0, 255, 255]

        # Blend overlay with original
        blended = cv2.addWeighted(original_frame, 0.90, fullsize_overlay, 0.3, 0)

        ros_image = self.bridge.cv2_to_imgmsg(blended, encoding='bgr8')
        self.image_pub.publish(ros_image)
        self.get_logger().info("Published ground segmentation image")




def main(args=None):
    rclpy.init(args=args)
    node = GroundSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

