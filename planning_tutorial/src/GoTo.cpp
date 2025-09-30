//GoTo.cpp
#include "planning_tutorial/GoTo.h"
#include <chrono>

namespace GoTo{

SetGoal::SetGoal(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    // location = config.blackboard->get<std::string>("location");

    // client_ = node_->create_client<lifecycle_msgs::srv::ChangeState>("/semantic_navigator3/change_state");
    rb_client_ = node_->create_client<rby1_custom_interfaces::srv::CallLoc>("/navigate_to_semantic_region");

    
}

NodeStatus SetGoal::tick(){
    location = getInput<std::string>("location").value();

    RCLCPP_INFO(node_->get_logger(), "SetGoal %s: Calling ACTIVATE service...", location.c_str());

    // if(!client_->wait_for_service(std::chrono::seconds(1))){
    //     RCLCPP_ERROR(node_->get_logger(), "Service Lifecycle is not available");
    //     return NodeStatus::FAILURE;
    // }

    auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    while (true) {
        now = std::chrono::steady_clock::now();
        if (now - start > std::chrono::seconds(2)) {
            break;
        }
    }

    // auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    // request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

    // future_ = client_->async_send_request(request);

    if(!rb_client_->wait_for_service(std::chrono::seconds(5))){
        RCLCPP_ERROR(node_->get_logger(), "SetGoal is not available");
        return NodeStatus::FAILURE;
    }

    auto rb_request = std::make_shared<rby1_custom_interfaces::srv::CallLoc::Request>();
    rb_request->region_name = location;

    rb_future_ = rb_client_->async_send_request(rb_request);

    return NodeStatus::SUCCESS;
}

BT::PortsList SetGoal::providedPorts() {
    return 
    {
        BT::InputPort<std::string>("location", "where to go"),
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
    };
}


IsReached::IsReached(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/goal_reached", 10,
        std::bind(&IsReached::topic_callback, this, std::placeholders::_1)
    );
    goal_reached_ = false;
}

BT::PortsList IsReached::providedPorts()
{
    return 
    {
        BT::InputPort<rclcpp::Node::SharedPtr>("node") 
    };
}

NodeStatus IsReached::tick(){
    if (!initialized_){
        RCLCPP_INFO(node_->get_logger(), "IsReached initialized.");
        initialized_ = true;
        start = std::chrono::steady_clock::now();
        current = start;
        state = "navigating";
        return NodeStatus::RUNNING;
    }
    if (goal_reached_)
    {
        RCLCPP_INFO(node_->get_logger(), "Goal reached!");
        initialized_ = false;

        state == "idle";
        goal_reached_ = false;
    
        return NodeStatus::SUCCESS;
    }
    else if (std::chrono::steady_clock::now() - start > std::chrono::seconds(100)){
        RCLCPP_ERROR(node_->get_logger(), "Timeout while waiting for goal to be reached.");
        return NodeStatus::FAILURE;
    }
    else if  (!goal_reached_)
    {   
        if (std::chrono::steady_clock::now() - current > std::chrono::seconds(5)){
            RCLCPP_INFO(node_->get_logger(), "Goal reached not yet.");
        }
        
        return NodeStatus::RUNNING;
    }
    
}

void IsReached::topic_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if (state == "navigating"){
        goal_reached_ = (msg->data);
    }
    else if (state == "idle"){
        goal_reached_ = false;
    }
    else{
        RCLCPP_ERROR(node_->get_logger(), "Unknown state received: %s", state.c_str());   
    } 
}

void RegisterNodes(BT::BehaviorTreeFactory& factory){
    factory.registerNodeType<SetGoal>("SetGoal");
    factory.registerNodeType<IsReached>("IsReached");
}
}