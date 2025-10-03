import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum, auto


# FSM State 정의
class State(Enum):
    START = auto()
    INITIALIZE = auto()
    DETECT_DOOR = auto()
    GOTODOOR = auto()
    OPENDOOR = auto()
    GREETINGS = auto()
    TAKEPICTURE = auto()
    TTSFORGUEST = auto()
    ASKNAME = auto()
    GOTOKITCHEN = auto()
    TTSFORGUEST2 = auto()
    ASKFAVORITE = auto()
    FINDFAVORITE = auto()
    ASKINTEREST = auto()
    GOTOLIVINGROOM = auto()
    FINDHOST = auto()
    FINDREMAININGCHAIR = auto()
    POINTCHAIR = auto()
    # 복구 상태
    CHANGING_POSE = auto()
    REQUESTING_HANDSUP = auto()
    REQUESTING_VOICE = auto()
    ERROR = auto()


# 완료 조건 정의
COMPLETION_TRIGGERS = {
    State.INITIALIZE: {"INITIALIZEDONE"},
    State.DETECT_DOOR: {"found"},
    State.GOTODOOR: {"arrived"},
    State.OPENDOOR: {"opened door"},
    State.ASKNAME: {"ASKNAMEDONE"},
    State.TTSFORGUEST: {"TTSFORGUESTDONE"},
    State.ASKFAVORITE: {"ASKFAVORITEDONE"},
    State.TTSFORGUEST2: {"got interests"},
    State.FINDFAVORITE: {"found"},
    State.ASKINTEREST: {"ASKINTERESTDONE"},
    State.GOTOKITCHEN: {"arrived"},
    State.GOTOLIVINGROOM: {"arrived"},
    State.FINDHOST: {"found"},
    State.FINDREMAININGCHAIR: {"found"},
    State.POINTCHAIR: {"pointed"},
}


class FSMNode(Node):
    def __init__(self):
        super().__init__("fsm_node")
        self.get_logger().info("FSM Node started")

        # FSM 초기 상태
        self.current_state = State.INITIALIZE

        # 퍼블리셔: 현재 FSM 상태
        self.state_pub = self.create_publisher(String, "/fsm/current_state", 10)

        # 구독자: /mission/status
        self.create_subscription(String, "/mission/status", self.mission_status_callback, 10)

        # 최초 상태 퍼블리시
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.current_state.name
        self.state_pub.publish(msg)
        self.get_logger().info(f"[FSM] Published state: {self.current_state.name}")

    def mission_status_callback(self, msg: String):
        status = msg.data.strip()
        self.get_logger().info(f"[FSM] Received mission status: {status}")

        # 실패 처리
        if status.lower() == "failed":
            self.get_logger().warn(f"[FSM] State {self.current_state.name} failed. Moving to ERROR.")
            self.current_state = State.ERROR
            self.publish_state()
            return

        # 성공 조건 체크
        expected_triggers = COMPLETION_TRIGGERS.get(self.current_state, set())
        if status in expected_triggers:
            next_state = self.get_next_success_state()
            if next_state:
                self.current_state = next_state
                self.publish_state()
        else:
            self.get_logger().warn(f"[FSM] Unexpected status '{status}' for state {self.current_state.name}")

    def get_next_success_state(self):
        success_flow = {
            State.INITIALIZE: State.ASKNAME,
            #State.ASKNAME: State.GOTOKITCHEN,
            State.ASKNAME: State.ASKFAVORITE,
            #State.GOTOKITCHEN: State.TTSFORGUEST2,
            #State.TTSFORGUEST2: State.ASKFAVORITE,
            #State.ASKFAVORITE: State.FINDFAVORITE,
            State.ASKFAVORITE: State.ASKINTEREST,
            State.ASKINTEREST: State.GOTOLIVINGROOM,
            #State.FINDFAVORITE: State.GOTOLIVINGROOM,
            State.GOTOLIVINGROOM: State.FINDHOST,
            State.FINDHOST: State.FINDREMAININGCHAIR,
            State.FINDREMAININGCHAIR: State.POINTCHAIR,
            State.POINTCHAIR: State.INITIALIZE,  # 다시 시작
        }
        return success_flow.get(self.current_state, None)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

