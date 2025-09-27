//GoTo.cpp
#include "planning_tutorial/GoTo.h"
#include <chrono>

namespace GoTo{

SetGoal::SetGoal(const std::string& name, const BT::Nodeconfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    client_ = node_->create_client<lifecycle_msgs::srv::ChangeState>("/semantic_navigator3/change_state");
    rb_client_ = node_->create_client<rby1_custom_interfaces::srv::CallLoc>("/navigate_semantic_region");
}

NodeStatus SetGoal::OnStart(){
    RCLCPP_INFO(node_->get_logger(), "SetGoal : Calling ACTIVATE service...")

    if(!client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_ERROR(node_->get_logger(), "Service Lifecycle is not available");
        return NodeStatus::FAILURE;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

    future_ = client_->async_send_request(request);

    if(!rb_client->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_ERROR(node_->get_logger(), "SetGoal is not available");
        return NodeStatus::FAILURE;
    }

    //auto rb_request = std::make_shared<rby1_custom_interfaces::srv::CallLoc>();

    return NodeStatus::RUNNING;
}

NodeStatus SetGoal::OnRunning(){
    RCLCPP_INFO(node_->get_logger(), "SetGoal is running...");
    return NodeStatus::SUCCESS;
}

void SetGoal::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "SetGoal is halted!");
}

IsReached::IsReached(const std::string& name, const BT::Nodeconfig& config)
  : BT::SyncActionNode(name, config)
{
public:
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    node_->create_subscription<std_msgs::msg::Bool>(
        "/goal_reached", 10,
        std::bind(&IsReached::topic_callback, this, std::placeholders::_1)
    )
private:
    auto status= NodeStatus::RUNNING;

}
NodeStatus IsReached::tick(){
    return status;
}
BT::PortsList IsReached::providedPorts(){return {};}

void IsReached::topic_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if(msg->data == true){
        status = NodeStatus::SUCCESS;
    }
    else{
        status = NodeStatus::FAILURE;
    }
}


Speak::Speak(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
NodeStatus Speak::tick() { 
    //RCLCPP_INFO(rclcpp::get_logger("Speak"), "Speaking result...");
    
    RCLCPP_INFO(rclcpp::get_logger("Speak"), "I found it!");
    return NodeStatus::SUCCESS; 
}
BT::PortsList Speak::providedPorts(){ return {}; }

void RegisterNodes(BT::BehaviorTreeFactory& factory){
    factory.registerNodeType<SetGoal>("SetGoal");
    factory.registerNodeType<IsReached>("IsReached");
    factory.registerNodeType<Speak>("Speak");
}
}