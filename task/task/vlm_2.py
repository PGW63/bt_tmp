import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rby1_custom_interfaces.srv import SetCaptionMode
from ament_index_python import get_package_share_directory

import base64
import requests
import logging
from io import BytesIO
from PIL import Image as PILImage

log = logging.getLogger("AutoLLMCaptionerNode")
log.setLevel(logging.INFO)


class AutoLLMCaptionerNode(Node):
    def __init__(self):
        super().__init__('auto_vlm_captioner_node')
        self.bridge = CvBridge()

        # 기본 파라미터
        self.mode = 1
        self.object_name = None
        self.latest_image = None
        self.text_prompt = None

        # ROS 통신
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        self.text_prompt_sub = self.create_subscription(
            String, "/vlm/text_prompt", self.text_prompt_callback, 10
        )

        self.service = self.create_service(
            SetCaptionMode, "set_caption_mode", self.set_caption_mode_callback
        )

        # LLM API 세팅
        self.llm_api_url = "http://127.0.0.1:11111"
        self.llm_api_key = "ollama"
        self.llm_api_model_name = "llava:7b"

        self.get_logger().info("AutoLLMCaptionerNode started.")

    # ------------------ ROS 콜백 ------------------
    def image_callback(self, msg: Image):
        """ROS 이미지 토픽 수신"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def text_prompt_callback(self, msg: String):
        """텍스트 프롬프트 토픽 수신"""
        self.text_prompt = msg.data
        self.get_logger().info(f"텍스트 프롬프트 수신: {self.text_prompt}")

    def set_caption_mode_callback(self, request, response):
        """모드 및 객체 설정"""
        self.mode = request.mode
        self.object_name = request.object_name if request.object_name else None

        if self.mode == 4:  # 텍스트 입력 모드
            if not self.text_prompt:
                response.success = False
                response.message = "텍스트 프롬프트가 아직 수신되지 않았습니다."
                return response
            caption = self.process_text()
            response.success = True
            response.message = caption
            return response

        # 이미지 기반 모드
        if self.latest_image is None:
            response.success = False
            response.message = "아직 이미지가 수신되지 않았습니다."
            return response

        caption = self.process_image()
        response.success = True
        response.message = caption
        return response

    # ------------------ LLM 호출 로직 ------------------
    def process_image(self):
        if self.latest_image is None:
            return "이미지를 수신하지 못했습니다."

        # OpenCV 이미지를 JPEG → base64 변환
        pil_image = PILImage.fromarray(self.latest_image[:, :, ::-1])  # BGR→RGB
        buffer = BytesIO()
        pil_image.save(buffer, format='JPEG')
        base64_image = base64.b64encode(buffer.getvalue()).decode("utf-8")

        json_payload = self.get_request_json(base64_image)
        caption = self.call_llm(json_payload)
        self.get_logger().info(f"캡션 결과: {caption}")
        return caption

    def process_text(self):
        """텍스트 입력 모드 처리 (mode=4)"""
        json_payload = self.get_request_json_for_text(self.text_prompt)
        caption = self.call_llm(json_payload)
        self.get_logger().info(f"텍스트 설명 결과: {caption}")
        return caption

    def get_request_json(self, base64_image):
        system_prompt, user_prompt = self.get_prompts()

        messages = [
            {'role': 'system', 'content': system_prompt},
            {"role": "user", "content": [
                {"type": "text", "text": user_prompt},
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
                }
            ]}
        ]

        payload = {
            'model': self.llm_api_model_name,
            'messages': messages,
            'max_tokens': 300,
            'temperature': 0.1,
            'top_p': 0.9,
            'stream': False,
        }
        return payload

    def get_request_json_for_text(self, text):
        system_prompt, _ = self.get_prompts()

        messages = [
            {'role': 'system', 'content': system_prompt},
            {'role': 'user', 'content': text}
        ]

        payload = {
            'model': self.llm_api_model_name,
            'messages': messages,
            'max_tokens': 300,
            'temperature': 0.1,
            'top_p': 0.9,
            'stream': False,
        }
        return payload

    def get_prompts(self):
        if self.mode == 1:
            return (
                "You are an AI assistant that provides detailed captions for images. Answer in Korean.",
                ""
            )
        elif self.mode == 2:
            if self.object_name:
                return (
                    "You are an AI assistant that provides detailed captions for images. Answer in Korean.",
                    f"Explain the absolute and relative position of {self.object_name} in the image."
                )
            else:
                return (
                    "You are an AI assistant that provides detailed captions for images. Answer in Korean.",
                    "Generate a detailed Korean caption for this image."
                )
        elif self.mode == 3:
            return (
                "You are introducing this person to someone else. Describe only their appearance and unique traits in a way that remains recognizable even after some time, focusing on stable details such as clothing, hairstyle, and physical features. Do not mention the location, objects, or surroundings."
,
                ""
            )
        elif self.mode == 4:
            return (
                "You are a word extractor. Explain only the prompt below clearly and concisely in English. \
                You will receive a sentence and an additional instruction describing what to extract or answer from that sentence. \
                Follow the instruction strictly and output only the required word or phrase. \
                For example: \
                'My name is Alisa' and 'Only get name' → Answer: 'Alisa' \
                'I like the cola most' and 'Get his favorite drink' → Answer: 'cola'",

                ""  # user 입력은 text_prompt_callback에서 받아옴
            )
        else:
            return ("", "")

    def call_llm(self, json_payload):
        full_api_url = self.llm_api_url
        if not full_api_url.endswith('/'):
            full_api_url += '/'
        
        if 'google' in full_api_url:
            full_api_url += f'models/{self.llm_api_model_name}:generateContent'
            headers = {'Content-Type': 'application/json', 'x-goog-api-key': self.llm_api_key}
        else:
            # LM Studio는 /v1/chat/completions 엔드포인트를 사용합니다.
            full_api_url += 'v1/chat/completions'
            headers = {'Content-Type': 'application/json', 'Authorization': f'Bearer {self.llm_api_key}'}
            
        try:
            log.info(f"LLM API로 요청 중: {full_api_url}")
            response = requests.post(full_api_url, headers=headers, json=json_payload)
            response.raise_for_status()

            completion_json = response.json()
            if 'google' in full_api_url:
                result = completion_json['candidates'][0]['content']['parts'][0]['text']
            else:
                result = completion_json['choices'][0]['message']['content']
            
            return result.replace('\n', ' ')

        except requests.exceptions.RequestException as e:
            log.error(f"API 호출 실패: {e}")
            return "오류: API 호출에 실패했습니다."
        except (KeyError, IndexError) as e:
            log.error(f"API 응답 구조가 유효하지 않습니다: {e}")
            return "오류: 유효하지 않은 API 응답입니다."


def main(args=None):
    rclpy.init(args=args)
    node = AutoLLMCaptionerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
