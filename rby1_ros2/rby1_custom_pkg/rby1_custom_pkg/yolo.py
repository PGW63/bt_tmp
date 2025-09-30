#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json

class YoloV8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')

        self.model = YOLO('knob.pt')  # 🔁 경로 수정
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(String, '/yolov8_detections', 10)
        self.get_logger().info('YOLOv8 node started with custom weights')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame)[0]

        detections = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].tolist()

            detections.append({
                'class_id': cls_id,
                'confidence': conf,
                'bbox': xyxy
            })

        # Publish as JSON string
        detection_msg = String()
        detection_msg.data = json.dumps(detections)
        self.publisher_.publish(detection_msg)

        # Optional: show result
        annotated = results.plot()
        cv2.imshow("YOLOv8 Detection", annotated)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

