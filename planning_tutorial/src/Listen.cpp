#include "planning_tutorial/Listen.h"

namespace Listen
{
    
Listen::Listen(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");


    sub_ = node_->create_subscription<std_msgs::msg::String>(
        "valid_text", 10,
        std::bind(&Listen::topic_callback, this, std::placeholders::_1)
    );

    pub_ = node_->create_publisher<std_msgs::msg::String>("is_listening", 10);



}

BT::PortsList Listen::providedPorts()
{
    return
    {
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
        BT::OutputPort<std::string>("heard_text", "Heard text")
    };    

}

NodeStatus Listen::onStart()
{   
    received_text_ = false;
    auto msg = std_msgs::msg::String();
    msg.data = "listen";
    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Listen started.");
    return NodeStatus::RUNNING;
}

NodeStatus Listen::onRunning()
{
    if (received_text_){
        RCLCPP_INFO(node_->get_logger(), "Listen succeeded.");
        return NodeStatus::SUCCESS;
    }
    else
    {
        return NodeStatus::RUNNING;
    }
}

void Listen::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "Listen HALTED");
    received_text_ = false;
}

void Listen::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    received_text_ = true;
    RCLCPP_INFO(node_->get_logger(), "Heard text: %s", msg->data.c_str());
    setOutput("heard_text", msg->data);
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<Listen>("Listen");
}


} // namespace Listen



