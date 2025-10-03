#include "planning_tutorial/Speak.h"
#include <chrono>

namespace Speak {

Speak::Speak(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    client_ = rclcpp_action::create_client<TTS>(node_, "/say");
}

BT::PortsList Speak::providedPorts(){
    return {
        BT::InputPort<std::string>("text", "Text to be spoken"),
        BT::InputPort<rclcpp::Node::SharedPtr>("node")
    };
}

BT::NodeStatus Speak::onStart(){
    auto text = getInput<std::string>("text");
    if(!text){
        RCLCPP_ERROR(node_->get_logger(), "missing input [text]");
        return BT::NodeStatus::FAILURE;
    }

    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available");
        return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = TTS::Goal();
    goal_msg.text = text.value();

    RCLCPP_INFO(node_->get_logger(), "Speak: sending goal...");

    auto send_goal_options = rclcpp_action::Client<TTS>::SendGoalOptions();
    auto future_goal_handle = client_->async_send_goal(goal_msg, send_goal_options);

    // goal_handle_future 저장 (멤버 변수)
    future_goal_handle_ = std::move(future_goal_handle);

    finished_ = false;
    success_ = false;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Speak::onRunning(){
    // 아직 goal_handle 확인 안했으면 먼저 확인
    if (future_goal_handle_.valid() && !goal_handle_) {
        if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            goal_handle_ = future_goal_handle_.get();
            if (!goal_handle_) {
                RCLCPP_ERROR(node_->get_logger(), "Speak: Goal rejected");
                return BT::NodeStatus::FAILURE;
            }
            // 결과 future 준비
            future_result_ = client_->async_get_result(goal_handle_);
        }
        return BT::NodeStatus::RUNNING;
    }

    // 결과 기다리는 중
    if (future_result_.valid()) {
        if (future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto result = future_result_.get();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node_->get_logger(), "Speak: finished successfully");
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Speak: aborted or canceled");
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    return BT::NodeStatus::RUNNING;
}

void Speak::onHalted(){
    RCLCPP_WARN(node_->get_logger(), "Speak halted");
    finished_ = false;
    success_ = false;
    goal_handle_.reset();
}

void RegisterNodes(BT::BehaviorTreeFactory& factory){
    factory.registerNodeType<Speak>("Speak");
}

} // namespace Speak
