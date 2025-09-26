#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# YOLO World V2
from ultralytics import YOLOWorld

# -----------------------------------------------------------------
# YOLO World 실제 감지 로직
# -----------------------------------------------------------------
class YoloDetector:
    def __init__(self, logger):
        self.logger = logger
        try:
            model_path = "yolov8s-world.pt"  # 모델 경로
            self.model = YOLOWorld(model_path)
            self.logger.info(f"YOLO World 모델 로드 완료: {model_path}")
        except Exception as e:
            self.logger.error(f"YOLO World 모델 로드 실패: {e}")
            raise e

    def detect(self, cv_image):
        """
        cv_image (OpenCV BGR 이미지)를 받아 YOLO World 결과 반환
        """
        try:
            results = self.model.predict(cv_image, verbose=False)
            if not results:
                return None
            return results[0]
        except Exception as e:
            self.logger.error(f"YOLO 추론 중 에러: {e}")
            return None

# -----------------------------------------------------------------
# ROS2 라이프사이클 노드
# -----------------------------------------------------------------
class YoloDetectionNode(LifecycleNode):

    def __init__(self, **kwargs):
        super().__init__('yolo_node', **kwargs)
        self.target_object = "unknown"
        self.isdetected_pub = None
        self.debug_image_pub = None
        self.set_objects_sub = None
        self.image_sub = None
        self.detector = None
        self.bridge = CvBridge()
        self.get_logger().info("YoloDetectionNode가 생성되었습니다 (Unconfigured 상태).")

    # Configure
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring... (Inactive 상태 진입)")
        try:
            self.isdetected_pub = self.create_publisher(String, '/isdetected', 10)
            self.debug_image_pub = self.create_publisher(Image, '/yolo/debug_image', 10)
            self.set_objects_sub = self.create_subscription(String, '/set_objects', self.set_objects_callback, 10)
            
            # YOLO World 모델 로드
            self.detector = YoloDetector(self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Configure 실패: {e}")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Configure 성공.")
        return TransitionCallbackReturn.SUCCESS

    # Activate
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Activating... (Active 상태 진입)")
        try:
            self.image_sub = self.create_subscription(Image, '/rgb', self.image_callback, 10)
            self.get_logger().info('Image subscription started.')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activate 실패: {e}")
            return TransitionCallbackReturn.FAILURE

    # Deactivate
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating YoloDetectionNode...')
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info('Image subscription stopped.')
        return TransitionCallbackReturn.SUCCESS

    # Cleanup
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up... (Unconfigured 상태 복귀)")
        if self.isdetected_pub: self.destroy_publisher(self.isdetected_pub)
        if self.debug_image_pub: self.destroy_publisher(self.debug_image_pub)
        if self.set_objects_sub: self.destroy_subscription(self.set_objects_sub)
        if self.image_sub: self.destroy_subscription(self.image_sub)
        self.isdetected_pub = None
        self.debug_image_pub = None
        self.set_objects_sub = None
        self.image_sub = None
        self.detector = None
        self.get_logger().info("Cleanup 성공.")
        return TransitionCallbackReturn.SUCCESS

    # Shutdown
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    # /set_objects 콜백
    def set_objects_callback(self, msg):
        if msg.data != self.target_object:
            self.get_logger().info(f"새로운 타겟 수신: {msg.data}")
            self.target_object = msg.data

    # /rgb 콜백
    def image_callback(self, msg):
        if self.target_object in ["unknown", ""]:
            return
        
        status_msg = String()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            yolo_result = self.detector.detect(cv_image)
            
            detected_names = []
            if yolo_result and yolo_result.boxes:
                class_indices = yolo_result.boxes.cls.cpu().numpy()
                class_name_map = yolo_result.names
                for cls_id in class_indices:
                    detected_names.append(class_name_map[int(cls_id)])
            
            found = self.target_object in detected_names
            status_msg.data = "Success" if found else "Detecting"
            if self.isdetected_pub:
                self.isdetected_pub.publish(status_msg)
            
            if yolo_result and self.debug_image_pub:
                debug_image = yolo_result.plot()
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"감지 중 예외: {e}")
            status_msg.data = "Failure"
            if self.isdetected_pub:
                self.isdetected_pub.publish(status_msg)

# -----------------------------------------------------------------
# main 함수
# -----------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    yolo_node = YoloDetectionNode()
    executor.add_node(yolo_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
