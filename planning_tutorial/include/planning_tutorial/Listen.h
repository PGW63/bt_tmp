// Listen.h

#ifndef PLANNING_TUTORIAL_LISTEN_H
#define PLANNING_TUTORIAL_LISTEN_H
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <atomic>
#include <memory>

namespace Listen
{
using BT::NodeStatus;

class Listen : public BT::StatefulActionNode
{
public:
    Listen(const std::string& name, const BT::NodeConfig& config);
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    static BT::PortsList providedPorts();
private:
    rclcpp::Node::SharedPtr node_;
    bool received_text_ = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

};

void RegisterNodes(BT::BehaviorTreeFactory& factory);
}

#endif

