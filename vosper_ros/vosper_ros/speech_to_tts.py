import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from audio_common_msgs.action import TTS


class SpeechToTTS(Node):
    def __init__(self):
        super().__init__('speech_to_tts')

        # /speech_text 구독
        self.subscription = self.create_subscription(
            String,
            'speech_text',
            self.listener_callback,
            10
        )

        # /say 액션 클라이언트 생성
        self._client = ActionClient(self, TTS, '/say')

        # TTS 상태 퍼블리셔 (playing / done)
        self.tts_state_pub = self.create_publisher(String, 'tts_state', 10)

        self.get_logger().info("speech_to_tts node started. Waiting for speech...")

    def publish_state(self, state: str):
        """TTS 상태 퍼블리시"""
        msg = String()
        msg.data = state
        self.tts_state_pub.publish(msg)
        self.get_logger().info(f"[TTS_STATE] {state}")

    def listener_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"Received text: {text}")

        # 액션 서버 준비될 때까지 대기
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("TTS server not available")
            return

        # TTS goal 생성
        goal_msg = TTS.Goal()
        goal_msg.text = text

        # TTS 시작 알림
        self.publish_state("playing")

        # 액션 전송
        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("TTS goal rejected")
            self.publish_state("done")  # 실패했어도 상태는 done
            return

        self.get_logger().info("Sent TTS request")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("TTS finished")
        # TTS 종료 알림
        self.publish_state("done")


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTTS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

