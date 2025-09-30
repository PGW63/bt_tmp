import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from rby1_custom_interfaces.srv import CallLoc

class SemanticNavigator(Node):

    def __init__(self):
        super().__init__('semantic_navigator')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)

        # /odom 토픽 구독자로 변경
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile
        )

        self.service = self.create_service(
            CallLoc,
            'navigate_to_semantic_region',
            self.navigate_to_semantic_region_callback
        )
        
        self.robot_pose = None
        self.get_logger().info("Semantic Navigation Service is ready.")

        # 맵 파일 로드
        self.semantic_map = self.load_semantic_map('/home/user/rby1_ros2_ws2/src/rby1_custom_pkg/configs/semantic.yaml') 
        if self.semantic_map is None:
            self.get_logger().error("Failed to load semantic map.")
            return

    def load_semantic_map(self, file_path):
        try:
            with open(file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file: {e}")
            return None

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position

    def is_inside_polygon(self, point, polygon):
        # 기존 코드와 동일
        x, y = point.x, point.y
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside
    
    def get_closest_point_in_polygon(self, current_pose, polygon):
        # 기존 코드와 동일
        closest_point = None
        min_dist_sq = float('inf')
        for point in polygon:
            dist_sq = (point[0] - current_pose.x)**2 + (point[1] - current_pose.y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_point = point
        return closest_point

    def get_closest_passage(self, current_pose, target_passage):
        # 기존 코드와 동일
        closest_point = None
        min_dist_sq = float('inf')
        for point in target_passage:
            dist_sq = (point[0] - current_pose.x)**2 + (point[1] - current_pose.y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_point = point
        return closest_point

    def navigate_to_semantic_region_callback(self, request, response):
        region_name = request.region_name
        self.get_logger().info(f"Received request to navigate to: {region_name}")

        if self.robot_pose is None:
            response.success = False
            response.message = "Robot pose is not available. Waiting for odom."
            self.get_logger().warn(response.message)
            return response

        target_region = None
        for region in self.semantic_map.get('semantic_regions', []):
            if region.get('tags') == region_name:
                target_region = region
                break
        
        if not target_region:
            response.success = False
            response.message = f"Region '{region_name}' not found in the map."
            self.get_logger().error(response.message)
            return response
        
        current_zone_polygon = target_region.get('zone', [])
        if self.is_inside_polygon(self.robot_pose, current_zone_polygon):
            response.success = True
            response.message = f"Robot is already in the '{region_name}'."
            self.get_logger().info(response.message)
            return response

        target_passage = target_region.get('passage', [])
        if not target_passage:
            response.success = False
            response.message = f"Passage data not found for region '{region_name}'."
            self.get_logger().error(response.message)
            return response

        goal_point = self.get_closest_passage(self.robot_pose, target_passage)
        
        if not goal_point:
            response.success = False
            response.message = "Could not find a valid goal point."
            self.get_logger().error(response.message)
            return response

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = "map"
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_point[0]
        goal_pose_msg.pose.position.y = goal_point[1]
        goal_pose_msg.pose.orientation.w = 1.0
        
        self.goal_publisher.publish(goal_pose_msg)
        self.get_logger().info(f"Published goal to x={goal_point[0]}, y={goal_point[1]} to reach '{region_name}'")

        response.success = True
        response.message = f"Navigating to the closest passage of '{region_name}'."
        return response

def main(args=None):
    rclpy.init(args=args)
    semantic_navigator = SemanticNavigator()
    rclpy.spin(semantic_navigator)
    semantic_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()