// FindObjects.cpp
#include "planning_tutorial/FindObjects.h"
#include <chrono> 

namespace FindObjects {

// ----------------- SetObjects -----------------
SetObjects::SetObjects(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), 
      current_state_(State::SERVICE_CALL)
{
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();

    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    pub_ = node_->create_publisher<std_msgs::msg::String>("/set_objects", qos);
    
    // yolo_node lifecycle 관리용 클라이언트
    client_ = node_->create_client<lifecycle_msgs::srv::ChangeState>("/yolo_node/change_state");
}

NodeStatus SetObjects::onStart() {
    current_state_ = State::SERVICE_CALL;
    RCLCPP_INFO(node_->get_logger(), "SetObjects: Calling ACTIVATE service...");

    // 서비스가 준비되었는지 확인
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service /yolo_node/change_state not available");
        return NodeStatus::FAILURE;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    // [중요] CONFIGURE → ACTIVATE 로 변경
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE; 
    
    future_ = client_->async_send_request(request);
    current_state_ = State::WAITING_FOR_RESPONSE;

    return NodeStatus::RUNNING; // 서비스 응답 대기
}

NodeStatus SetObjects::onRunning() {
    if (current_state_ == State::WAITING_FOR_RESPONSE) {
        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = future_.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "Activate service SUCCESS. Publishing target object...");
                
                auto text = getInput<std::string>("willfinds");
                if (!text) {
                    RCLCPP_ERROR(node_->get_logger(), "missing input [willfinds]");
                    return NodeStatus::FAILURE;
                }
                auto msg = std_msgs::msg::String();
                msg.data = text.value();
                pub_->publish(msg);

                return NodeStatus::SUCCESS; 
                
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Activate service FAILED");
                return NodeStatus::FAILURE;
            }
        } else {
            return NodeStatus::RUNNING; 
        }
    }
    return NodeStatus::RUNNING;
}

void SetObjects::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "SetObjects HALTED");
    current_state_ = State::SERVICE_CALL;
}

BT::PortsList SetObjects::providedPorts() {
    return { BT::InputPort<std::string>("willfinds") };
}


// ----------------- IsDetected -----------------
IsDetected::IsDetected(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), 
      atomic_status_(NodeStatus::RUNNING)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/isdetected", 10,
        std::bind(&IsDetected::topic_callback, this, std::placeholders::_1)
    );
}

BT::PortsList IsDetected::providedPorts(){
    return {};
}

NodeStatus IsDetected::onStart() {
    atomic_status_.store(NodeStatus::RUNNING);
    RCLCPP_INFO(node_->get_logger(), "IsDetected: Waiting for detection result...");
    return NodeStatus::RUNNING;
}

NodeStatus IsDetected::onRunning() {
    // topic_callback에서 바꾼 상태 반환
    return atomic_status_.load(); 
}

void IsDetected::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "IsDetected HALTED");
    atomic_status_.store(NodeStatus::RUNNING);
}

void IsDetected::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "Success") {
        atomic_status_.store(NodeStatus::SUCCESS);
    } else if (msg->data == "Detecting") {
        atomic_status_.store(NodeStatus::RUNNING);
    } else {
        atomic_status_.store(NodeStatus::FAILURE);
    }
}


// ----------------- Pan -----------------
Pan::Pan(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
NodeStatus Pan::tick() { 
    RCLCPP_INFO(rclcpp::get_logger("Pan"), "Panning camera...");
    return NodeStatus::SUCCESS; 
}
BT::PortsList Pan::providedPorts(){ return {}; }


// ----------------- Speak -----------------
Speak::Speak(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
NodeStatus Speak::tick() { 
    //RCLCPP_INFO(rclcpp::get_logger("Speak"), "Speaking result...");
    
    RCLCPP_INFO(rclcpp::get_logger("Speak"), "I found it!");
    return NodeStatus::SUCCESS; 
}
BT::PortsList Speak::providedPorts(){ return {}; }


// ----------------- RegisterNodes -----------------
void RegisterNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<SetObjects>("SetObjects");
    factory.registerNodeType<IsDetected>("IsDetected");
    factory.registerNodeType<Pan>("Pan");
    factory.registerNodeType<Speak>("Speak");
}

} // namespace FindObjects
