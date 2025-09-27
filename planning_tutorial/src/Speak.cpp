// Speack.cpp

#include "planning_tutorial/Speak.h"
#include <chrono>

namespace Speack{

Speak::Speak(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    std::string server_name = "/say";

    this->client_ = rclcpp_action::create_client<TTS>(node_, server_name);
}

BT::PortsList Speak::providedPorts(){
    return { BT::InputPort<std::string>("text", "Text to be spoken") };
}

NodeStatus Speak::onStart(){
    auto text = getInput<std::string>("text","I did it!");
    if(!text){
        RCLCPP_ERROR(node_->get_logger(),"missing input [text]");
        return NodeStatus::FAILURE;
    }

    if (!this->client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        return NodeStatus::FAILURE;
    }

    auto goal_msg = TTS::Goal();
    goal_msg.text = text.value();

    RCPCPP_INFO(node_->get_logger(), "sending goal...");

    auto send_goal_options = rclcpp_action::Client<TTS>::SendGoalOptions();

    auto goal_handle_future = this->client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp:spin_until_future_complete(node_, goal_handle_future, std::chrono::seconds(1))) != rclcpp::FutureReturnCode::SUCCESS
    {
        RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
        return NodeStatus::FAILURE;
    }

    GoalHandleTTS::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle){
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        return NodeStatus::FAILURE;
    }

    future_result_ = this->client_->asynce_get_result(goal_handle);
    return NodeStatus::RUNNING;
}

NodeStatus Speak::onRunning(){
    if (future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto result = future_result_.get();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(), "TTS succeeded");
            return NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "TTS was aborted or canceled");
            return NodeStatus::FAILURE;
        }
    }

    return NodeStatus::RUNNING;
}

void Speak::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "Speak halted");
}

void RegisterNodes(BT::BehaviorTreeFactory& factory){
    factory.registerNodeType<Speak>("Speak");
}

}