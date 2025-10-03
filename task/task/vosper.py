import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
import threading
import os
import time
import re
import vosper
from ament_index_python import get_package_share_directory

class VosperNode(Node):
    def __init__(self):
        super().__init__('vosper_node')

        self.get_logger().info("Initializing VosperNode...")
        self.config_path = os.path.join(get_package_share_directory('task'),
                                        'config', 'vosper_config.yaml')
        
        self.data = self.load_yaml(self.config_path)

        self.speech_pub = self.create_publisher(String, 'speech_text', 10)
        self.state_pub = self.create_publisher(String, 'speech_state', 10)
        self.name_pub = self.create_publisher(String, '/face/register_name', 10)

        self.is_tts_playing = False
        self.create_subscription(String, 'tts_state', self.tts_state_callback, 10)
        self.create_subscription(String, 'current_state', self.current_state_callback, 10)

        self.count = 0
        self.last_answer=""
        self.current_state = None

        self.vosper_instance = vosper.new(
            vosk_model='small',
            whisper_model='small.en',
            waiting_time=2,
            filename='speaker',
            verbosity=True,
        )

        self.re_question = "is it correct?"
        self.last_text = ""
        self.last_time = 0.0

        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.thread.start()
        self._tts_lock = threading.Lock()

    def tts_state_callback(self, msg):
        """ TTS state callback"""
        with self._tts_lock:
            if msg.data == "playing":
                self.is_tts_playing = True
            elif msg.data == "finished":
                self.is_tts_playing = False
        
        self.get_logger().info(f"[TTS] TTS is playing : {self.is_tts_playing}")
    
    def current_state_callback(self, msg):
        """ current state callback """
        self.current_state = msg.data
        self.get_logger().info(f"[STATE] Current state updated to: {self.current_state}")

    def load_yaml(self, path):
        """ Load YAML configuration """
        try:
            with open(path, 'r') as file:
                data = yaml.safe_load(file)
                self.get_logger().info(f"YAML configuration loaded from {path}")
                return data
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found: {path}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file: {e}")
            return None
    
    def listen_loop(self):
        """ vosper.listin() 루프 """
        while rclpy.ok():
            with self._tts_lock:
                tts_playing = self.is_tts_playing
            if tts_playing:
                self.vosper_instance.reset()
                continue
            
            while self.is_tts_playing:
                time.sleep(0.3)
            
            say_text = load_text(self.current_state)

            text = self.vosper_instance.listen()

            if not text:
                continue

            if text.endswith('.') or text.endswith('?') or text.endswith('!'):
                self.get_logger().info(text)
                first_values = []
                for k, v in self.data.items():
                    if isinstance(v, list) and v:
                        first_values.append(v[0])
                
                texts = re.sub(r'[^a-zA-Z\s]', '', text)
                tokens = texts.lower().split()

                if not tokens:
                    self.get_logger().warning(f"Empty tokens after cleaning: {text}")
                    continue
                
                self.get_logger().info(f"\r [whisper] {text}")
                self.last_answer = tokens[-1]

                # FSM state check, each state별로 동작 다르게

            elif len(text.split()) < 2 and text.lower() == "yes":
                pass

            elif len(text.splot()) < 2 and text.lower() == "no":
                pass

            else:
                continue

            now = time.time()
            


