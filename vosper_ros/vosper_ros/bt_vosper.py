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
        super().__init__('vosper_node2')
        self.get_logger().info("Vosper ROS2 node started")

        # 퍼블리셔
        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        self.state_pub = self.create_publisher(String, 'speech_state', 10)
        self.text_pub = self.create_publisher(String, 'valid_text', 10)

        # TTS 상태 구독
        self.is_tts_playing = False
        self.create_subscription(String, 'tts_state', self.tts_state_callback, 10)
        
        self.start_msg = None
        self.create_subscription(String, 'is_listening', self.listen_callback, 10)

        # vosper 인스턴스
        self.vosper_instance = vosper.new(
            vosk_model='small',
            whisper_model='medium.en',
            waiting_time=4,
            filename='speaker',
            verbosity=True
        )

        self.valid_text = None

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

    def listen_callback(self, msg: String):
        """listening 시작 신호 수신"""
        self.start_msg = msg.data
        self.publish_state("🎤 이제 말씀을 해주세요.")

    def publish_state(self, state: str):
        """상태 메시지 퍼블리시"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"[STATE] {state}")

    def listen_loop(self):
        """vosper.listen()을 조건부로 실행"""
        while rclpy.ok():
            # 듣기 시작 신호가 없거나 TTS 재생 중이면 대기
            if self.start_msg != "listen" or self.is_tts_playing:
                time.sleep(0.1)
                continue
            
            # self.get_logger().warn("[Vosper] Listening...")
            text = self.vosper_instance.listen()
            if not text:
                continue
            

            if text.lower() == "yes." or text.lower() == "yes":
                self.get_logger().warn("[Vosper] Entered Yes")
                final_text = "Ok. I got it."
                self.text_pub.publish(String(data=self.valid_text))
                self.start_msg = None
                self.publish_state("listen is stopped")
                self.valid_text = None
                self.text = None
                self.last_text = None
                self.get_logger().error(f"[Vosper] {self.valid_text}")
                self.get_logger().error(f"[Vosper] {self.text}")

                

            elif text.lower() == "no." or text.lower() == "no":
                self.get_logger().warn("[Vosper] Entered No")
                final_text = "Then Please say again your order."
                self.valid_text = None

            # elif len(text.split()) < 2:
            #     continue

            ## 1) 문장이 구두점으로 끝나는 경우
            elif text.endswith('.') or text.endswith('?') or text.endswith('!'):
                self.get_logger().warn("[Vosper] Entered .")
                final_text = "did you say " + text + "?Please answer yes or no."
               
            else:
                self.get_logger().info(f"[Skip] {text}")
                continue  # 조건 안 맞으면 스킵
                
            # 중복 방지
            now = time.time()
            if final_text == self.last_text and (now - self.last_time) < 6.0:
                self.get_logger().warn("[Vosper] Entered 2222")
                continue
            

            self.valid_text = text
            self.last_text = final_text
            self.last_time = now
            
            os.system('clear')
            print(f"- {final_text}")
            self.get_logger().info(f"[FINAL] {final_text}")

            self.get_logger().warn("[Vosper] Entered ")

            msg = String()
            msg.data = final_text
            self.speech_pub.publish(msg)

            self.vosper_instance.reset()

            # 다음 발화 안내
            if self.start_msg == "listen":
                self.get_logger().warn("[Vosper] Entered 3333")
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
