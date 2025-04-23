#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference')
        self.bridge = CvBridge()
        
        # please replace the path with the actual path to your model
        self.model = YOLO('/home/chandhan/pallet_ws/src/pallet_segmentation/models/pallet_detection/best.pt')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/robot1/zed2i/left/image_rect_color', # Change topic name as needed
            self.image_callback,
            qos_profile)

        self.image_pub = self.create_publisher(
            Image, '/inference/pallet_detection', 10)

        self.get_logger().info("YOLOv11 Inference Node Started")
        
        
        
    def image_callback(self, msg):
        self.get_logger().info("Image received")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Initialize overlay
        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        combined_boxes = []

        result = self.model.predict(source=frame, conf=0.1, verbose=False)[0]

            # Combine masks
        if result.masks is not None:
            masks = result.masks.data.cpu().numpy()
            for mask in masks:
                combined_mask = np.maximum(combined_mask, (mask.astype(np.uint8) * 255))

            
        boxes = result.boxes
        if boxes is not None:
            xyxy = boxes.xyxy.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)
            for i, cls_id in enumerate(classes):
                if cls_id == 1: 
                    combined_boxes.append(list(map(int, xyxy[i])))


        # 🟢 Draw all combined boxes (green)
        for box in combined_boxes:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Publish
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(ros_image)
        self.get_logger().info("Published pallet image")
            
        

def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
