import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import threading
import time
import re
from ament_index_python.packages import get_package_share_directory # 이 줄을 추가
# vosper.py 경로 추가
vosper_path = os.path.expanduser('~/robocup_ws/src/vosper')
sys.path.append(vosper_path)

import vosper


class VosperNode(Node):
    def __init__(self):
        super().__init__('vosper_node_receptionist')
        self.get_logger().info("Vosper ROS2 node started")

        # YAML 파일 경로 로드
        config_path = os.path.join(
            get_package_share_directory('vosper_ros'),
            'config',
            'data.yaml'
        )

        # YAML 파일 로드
        try:
            with open(config_path, 'r') as file:
                self.data = yaml.safe_load(file)
                self.get_logger().info("YAML data loaded successfully.")
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found at {config_path}")
            self.data = None
            
        # 퍼블리셔
        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        self.state_pub = self.create_publisher(String, 'speech_state', 10)

        # TTS 상태 구독
        self.is_tts_playing = False
        self.create_subscription(String, 'tts_state', self.tts_state_callback, 10)

        # vosper 인스턴스
        self.vosper_instance = vosper.new(
            vosk_model='small',
            whisper_model='small.en',
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

            # 1) 문장이 구두점으로 끝나는 경우
            if text.endswith('.') or text.endswith('?') or text.endswith('!'):
                # 텍스트를 소문자로 변환하고 토큰화
                text = re.sub(r'[^a-zA-Z\s]', '', text)
                tokens = text.lower().split()

                matched_text = ""

                # 1. 키(Key) 매칭
                # YAML 데이터에 키(names, interests, drinks 등)가 있는지 확인
                matched_key = ""
                self.get_logger().info(f"Loaded YAML data: {self.data}")
                for token in tokens:
                    if token in self.data.keys():
                        matched_key = token
                        # 키가 매칭되면 해당 토큰과 그 이후 토큰들은 무시하고 값 매칭으로 넘어감
                        break
            
                # 키가 매칭된 경우, 해당 키의 값(리스트)에서 토큰 매칭
                if matched_key:
                    # 2. 값(Value) 매칭
                    # 키에 해당하는 리스트를 가져와서 텍스트의 첫 토큰부터 순차적으로 매칭
                    self.get_logger().info(f"Matched key found: {matched_key}. List: {self.data[matched_key]}")
                    for token1 in tokens:
                        if token1 in self.data[matched_key]:
                            # 텍스트의 토큰들이 값 항목과 순차적으로 일치하는지 확인
                            self.get_logger().info(f"token: {token1}. List: {self.data[matched_key]}")
                            matched_text = token1
                            continue
                        else:
                            self.get_logger().info(f"token: {token1} not in List: {self.data[matched_key]}")
                            pass
            
                # 최종 텍스트 발행
                if matched_text:
                    final_text = f"Your {matched_key} is {matched_text}. is it correct?"
                else:
                    final_text = "did you say" + text + " Sorry, I couldn't find a match. Please say again"

            # 2) 단답 "yes" 또는 "no"
            elif len(text.split()) < 2 and text.lower() == "yes":
                final_text = "Ok. Then I will Progress the Task 1."
              
            elif len(text.split()) < 2 and text.lower() == "no":
                final_text = "Then Please say again your order."
                
            else:
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

