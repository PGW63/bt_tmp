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
        self.config_path = os.path.join(
            get_package_share_directory('vosper_ros'),
            'config',
            'data.yaml'
        )

        # YAML 파일 로드	
        self.get_logger().info(f"YAML load path: {self.config_path}")
        self.data = self.load_yaml()
            
        # 퍼블리셔
        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        self.state_pub = self.create_publisher(String, 'speech_state', 10)
        self.name_pub = self.create_publisher(String, '/face/register_name', 10)  # MODIFIED: face register name 퍼블리시
        self.status_pub = self.create_publisher(String, '/mission/status', 10)   # ADDED: FSM 상태 완료 퍼블리시

        # TTS 상태 구독
        self.is_tts_playing = False
        self.create_subscription(String, 'tts_state', self.tts_state_callback, 10)
        
        # FSM 현재 상태 구독
        self.current_fsm_state = None  # ADDED
        self.create_subscription(String, '/fsm/current_state', self.fsm_state_callback, 10)  # ADDED
        self.count = 0
        self.last_answer = ""

        # vosper 인스턴스
        self.vosper_instance = vosper.new(
            vosk_model='small',
            whisper_model='small.en',
            waiting_time=4,
            filename='speaker',
            verbosity=True,
        )

        # 중복 방지용
        self.last_text = ""
        self.last_time = 0.0
        
        # listen() 별도 스레드 실행
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.thread.start()
        self._tts_lock = threading.Lock()
        
        self.publish_msg('receptionist task will be start soon')
        mission_success = "INITIALIZEDONE"
        status_msg = String()
        status_msg.data = mission_success
        self.status_pub.publish(status_msg)
        self.get_logger().info(f"[FSM SUCCESS] Published: {mission_success}")

    def tts_state_callback(self, msg: String):
        """TTS 상태 수신 (playing/done)"""
        with self._tts_lock:
            if msg.data == "playing":
                self.is_tts_playing = True
            elif msg.data == "done":
                self.is_tts_playing = False

        self.get_logger().info(f"[TTS] TTS is playing: {self.is_tts_playing}")
            
    def fsm_state_callback(self, msg: String):
        new_state = msg.data.strip()
        if new_state != self.current_fsm_state:  # 상태가 실제로 바뀌었을 때만
            self.current_fsm_state = new_state
            self.count = 0
            self.get_logger().info(f"[FSM] Current state updated: {self.current_fsm_state}")


    def publish_state(self, state: str):
        """상태 메시지 퍼블리시"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"[STATE] {state}")
        
    def load_yaml(self):
        """YAML 파일을 로드하는 헬퍼 함수"""
        try:
            with open(self.config_path, 'r') as file:
                data = yaml.safe_load(file)
                self.get_logger().info("YAML data loaded successfully.")
                return data
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found at {self.config_path}")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {e}")
            return None

    def save_yaml(self):
        """현재 self.data를 YAML 파일에 저장"""
        if self.data is None:
            self.get_logger().error("Cannot save YAML: data is None.")
            return

        # 데이터를 저장할 경로를 지정
        save_path = self.config_path # 또는 self.save_path
        
        try:
            with open(save_path, 'w') as file:
                yaml.safe_dump(self.data, file, default_flow_style=False)
            self.get_logger().info(f"YAML file saved at {save_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save YAML: {e}")
    
    def publish_msg(self, text: str):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
    
    def publish_name(self, text: str):
        msg = String()
        msg.data = text
        self.name_pub.publish(msg)

    def listen_loop(self):
        """vosper.listen()을 계속 실행해서 최종 문장만 발행"""
        while rclpy.ok():
            if self.current_fsm_state is None:
                continue
            # TTS 재생 중이면 듣지 않음
            with self._tts_lock:
                tts_playing = self.is_tts_playing
            if tts_playing:
                self.vosper_instance.reset()
                continue
            while self.is_tts_playing:  # TTS 끝날 때까지 기다림
                time.sleep(0.5)
    
            if self.current_fsm_state not in ['ASKNAME','ASKFAVORITE','ASKINTEREST']:
                continue  # 현재는 whisper가 필요한 노드가 아니므로 무시        
                  
            #self.get_logger().info(f"Checking ASKNAME: state={repr(self.current_fsm_state)}, count={self.count}")
            ################################################################################################################################################  TTS가 말하는 곳
            ############ 여기서부터 #########
            if self.current_fsm_state == 'GREETINGS':
                self.publish_msg('hello guest. plz just look at me with smile')
                continue
            
            elif self.current_fsm_state == 'TTSFORGUEST' and self.count == 0:
                #self.publish_msg('please answer after beep sound')
                self.publish_msg('hello guest. please just look at me with smile')
                mission_success = "TTSFORGUESTDONE"
                status_msg = String()
                status_msg.data = mission_success
                self.status_pub.publish(status_msg)
                self.get_logger().info(f"[FSM SUCCESS] Published: {mission_success}")
                self.count += 1
                continue
            
            elif self.current_fsm_state == 'TTSFORGUEST2':
                self.publish_msg('please stand behind me.') #please answer after beep sound
                continue
            
            elif self.current_fsm_state == 'GOTOKITCHEN':
                self.publish_msg('please follow me')
                continue
            
            elif self.current_fsm_state == 'GOTOLIVINGROOM':
                self.publish_msg('please follow me and stop behind me')
                continue
            ############ 여기까지는 FSM 통합 노드에서 발행하도록 하고 제외해도 됨 ######    
            
            elif self.current_fsm_state == 'ASKNAME' and self.count == 0:
                self.publish_msg('what is your name?')
                self.get_logger().info('what is your name?')
                self.count += 1
                continue
                
            elif self.current_fsm_state == 'ASKFAVORITE' and self.count == 0:
                self.publish_msg('what is your favorite drink?')
                self.get_logger().info('what is your favorite drink?')
                self.count += 1
                continue
                
            elif self.current_fsm_state == 'ASKINTEREST' and self.count == 0:
                self.publish_msg('what is your interest?')
                self.get_logger().info('what is your interest?')
                self.count += 1
                continue
                
                
            ##########################################################################################################################################3
                
            #####  음성인식 받아오는 곳
            text = self.vosper_instance.listen()
            #####
            if not text:
                continue
            # 1) 문장이 구두점으로 끝나는 경우
            if text.endswith('.') or text.endswith('?') or text.endswith('!'):
                #호스트 [이름, 취미, 음료]
                self.get_logger().info(text)
                first_values = []
                for k, v in self.data.items():
                    if isinstance(v, list) and v:   # v가 리스트이고 비어있지 않으면
                        first_values.append(v[0])   # 첫 번째 원소만 추가
                # 텍스트를 소문자로 변환하고 토큰화
                texts = re.sub(r'[^a-zA-Z\s]', '', text)
                tokens = texts.lower().split()
                if not tokens:
                    self.get_logger().warning(f"Empty tokens after cleaning: {text}")
                    continue
                # 👇 whisper 결과 실시간 표시 (반복 프린트 말고 업데이트)
                sys.stdout.write(f"\r[Whisper] {text}   ")  # 뒤에 공백 몇 개 넣어주면 이전 글자 잔상 제거
                sys.stdout.flush()
                self.get_logger().info(f"Checking {self.current_fsm_state}: count={self.count}")
                #matched_text = ""
                #matched_key = ""
                self.last_answer = tokens[-1]
                # FSM 상태 확인 후 State별로 처리                 
                if self.current_fsm_state == 'ASKNAME' and self.count == 1:
                    final_text = "your name is " + self.last_answer + " is it correct?"
                    self.count += 1
                                
                elif self.current_fsm_state == 'ASKFAVORITE' and self.count == 1:
                    final_text = "your favorite drink " + self.last_answer + " is it correct?"
                    self.count += 1
                                                 
                elif self.current_fsm_state == 'ASKINTEREST' and self.count == 1:
                    final_text = "your interest is " + self.last_answer + " is it correct?"
                    self.count += 1

            # 2) 단답 "yes" 또는 "no"
            elif len(text.split()) < 2 and text.lower() == "yes" and self.count == 2:
                mission_success = None  # FSM 성공 신호 담을 변수
                
                if self.current_fsm_state == 'ASKNAME':
                    if self.last_answer in first_values:
                        final_text = "Oh wow!. You have same name the " + self.last_answer + " as the host."
                    self.data["name"].append(self.last_answer)
                    self.publish_name(self.last_answer)
                    mission_success = "ASKNAMEDONE"
                    self.publish_msg(f'Your name {self.last_answer} has been registered in the guest list successfully.')
                    self.last_answer = ""
                    
                elif self.current_fsm_state == 'ASKFAVORITE':
                    if self.last_answer in first_values:
                        final_text = "Oh wow!. Your favorite drink the " + self.last_answer + " is same as the host."
                    self.data["drink"].append(self.last_answer)
                    mission_success = "ASKFAVORITEDONE"
                    self.publish_msg(f'Your favorite drink {self.last_answer} has been registered in the guest list successfully.')
                    self.last_answer = ""
                    
                elif self.current_fsm_state == 'ASKINTEREST':
                    if self.last_answer in first_values:
                        final_text = "Oh wow!. You have same interest the " + self.last_answer + " as the host."
                    self.data["interest"].append(self.last_answer)
                    mission_success = "ASKINTERESTDONE"
                    self.publish_msg(f'Your interest {self.last_answer} has been registered in the guest list successfully.')
                    self.last_answer = ""
                                        
                self.save_yaml()    
                self.count = 0     
                            
                # FSM 성공 메시지 퍼블리시
                if mission_success:
                    status_msg = String()
                    status_msg.data = mission_success
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f"[FSM SUCCESS] Published: {mission_success}")
                    
            elif len(text.split()) < 2 and text.lower() == "no" and self.count == 2:
                final_text = "Then Please say again."
                self.count = 0 
                self.last_answer = ""
                
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

