#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rby1_custom_interfaces.srv import CallLoc
from ament_index_python import get_package_share_directory


class SemanticNavigator(Node):
    """our nav2 goal_pose node (non-lifecycle)"""
    
    def __init__(self):
        super().__init__("semantic_navigator2")

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # 상태 변수
        self.robot_pose = None
        self.target_region = None
        self.target_passage = None
        self.closest_point = None
        self.goal_point = None

        # semantic.yaml 로드
        package_name = get_package_share_directory('rby1_custom_pkg')
        self.semantic_map = self.load_semantic_map(package_name + '/configs/semantic.yaml') 
        if self.semantic_map is None:
            self.get_logger().error("Failed to load semantic map.")
            return
        
        # Pub/Sub/Service/Action 초기화
        self.cli_ = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=ReentrantCallbackGroup())
        self.pub_ = self.create_publisher(Bool, "/goal_reached", 10)
        self.sub_ = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            self.qos_profile
        )
        self.ser_ = self.create_service(
            CallLoc,
            "navigate_to_semantic_region",
            self.nav_callback
        )

        self.get_logger().info("Semantic Navigator Node started.")

    def load_semantic_map(self, file_path):
        try:
            with open(file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file: {e}")
            return None
    
    def get_passage(self, current_robot_pose, target_passage):
        self.closest_point = None
        min_dist_sq = float('inf')
        for point in target_passage:
            dist_sq = (point[0] - current_robot_pose.x)**2 + (point[1] - current_robot_pose.y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                self.closest_point = point
        return self.closest_point
    
    def get_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal Rejected")
            return
        self.get_logger().info("Goal accepted")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        msg = Bool()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached")
            msg.data = True
        else:
            self.get_logger().info(f"Goal failed -> {status}")
            msg.data = False
        self.pub_.publish(msg)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg.pose.pose.position
    
    def nav_callback(self, request, response):
        region_name = request.region_name
        self.get_logger().info(f"Received {region_name}")

        if self.robot_pose is None:
            response.success = False
            self.get_logger().warn("robot pose is not received")
            return response
        
        self.target_region = None
        self.target_passage = None
        
        for region in self.semantic_map.get("semantic_regions", []):
            if region.get("tags") == region_name:
                self.target_region = region
                break
        
        if self.target_region is None:
            response.success = False
            response.message = f"Region '{region_name}' not found in YAML"
            self.get_logger().error(response.message)
            return response
        
        self.target_passage = self.target_region.get("passage", [])

        self.goal_point = self.get_passage(self.robot_pose, self.target_passage)

        if self.goal_point is None:
            response.success = False
            response.message = f"No valid passage point for region '{region_name}'"
            self.get_logger().error(response.message)
            return response
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_point[0]
        goal_msg.pose.pose.position.y = self.goal_point[1]
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.cli_.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.get_response_callback)

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
