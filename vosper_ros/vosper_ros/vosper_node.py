import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import threading
import time

# vosper.py 경로 추가
vosper_path = os.path.expanduser('~/robocup_ws/src/vosper')
sys.path.append(vosper_path)

import vosper


class VosperNode(Node):
    def __init__(self):
        super().__init__('vosper_node')
        self.get_logger().info("Vosper ROS2 node started")

        # 퍼블리셔
        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        self.state_pub = self.create_publisher(String, 'speech_state', 10)

        # TTS 상태 구독
        self.is_tts_playing = False
        self.create_subscription(String, 'tts_state', self.tts_state_callback, 10)

        # vosper 인스턴스
        self.vosper_instance = vosper.new(
            vosk_model='small',
            whisper_model='medium.en',
            waiting_time=4,
            filename='speaker',
            verbosity=True
        )

        # 중복 방지용
        self.last_text = ""
        self.last_time = 0.0
        
        # listen() 별도 스레드 실행
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.thread.start()

    def tts_state_callback(self, msg: String):
        """TTS 상태 수신 (playing/done)"""
        if msg.data == "playing":
            self.is_tts_playing = True
        elif msg.data == "done":
            self.is_tts_playing = False

    def publish_state(self, state: str):
        """상태 메시지 퍼블리시"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"[STATE] {state}")

    def listen_loop(self):
        """vosper.listen()을 계속 실행해서 최종 문장만 발행"""
        while rclpy.ok():
            if self.is_tts_playing:
                continue  # 현재는 TTS 출력 중이므로 무시

            text = self.vosper_instance.listen()
            if not text:
                continue

            ## 1) 문장이 구두점으로 끝나는 경우
            if text.endswith('.') or text.endswith('?') or text.endswith('!'):
                final_text = "did you say " + text + "?"

            # 2) 단답 "yes" 또는 "no"
            elif len(text.split()) < 2 and text.lower() == "yes":
                final_text = "Ok. Then I will Progress the Task 1."
              
            elif len(text.split()) < 2 and text.lower() == "no":
                final_text = "Then Please say again your order."
                
            else:
            	self.get_logger().info(f"[Skip] {text}")
            	continue  # 조건 안 맞으면 스킵
                
            # 중복 방지: 같은 텍스트가 2초 이내에 반복되면 무시
            now = time.time()
            if final_text == self.last_text and (now - self.last_time) < 6.0:
                continue

            # 기록 업데이트
            self.last_text = final_text
            self.last_time = now
            
            os.system('clear')
            print(f"- {final_text}")
            self.get_logger().info(f"[FINAL] {final_text}")

            msg = String()
            msg.data = final_text
            self.speech_pub.publish(msg)

            # 최종 문장 발행 후 → 다음 발화 안내
            self.publish_state("🎤 이제 다음 말씀을 해주세요.")


def main(args=None):
    rclpy.init(args=args)
    node = VosperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

