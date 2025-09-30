//GoTo.h

#ifndef PLANNING_TUTORIAL_GOTO_H
#define PLANNING_TUTORIAL_GOTO_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp" // <--- Add this include
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rby1_custom_interfaces/srv/call_loc.hpp"

namespace GoTo {

using BT::NodeStatus;

class SetGoal : public BT::SyncActionNode{
public:
    SetGoal(const std::string& name,const BT::NodeConfig& config);

    NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<rby1_custom_interfaces::srv::CallLoc>::SharedPtr rb_client_;
    // Correct future type
    std::shared_future<std::shared_ptr<rby1_custom_interfaces::srv::CallLoc::Response>> rb_future_;
    
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
    // Correct future type
    std::shared_future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>> future_;

    std::string location;
    
};

// Change base class to ConditionNode
class IsReached : public BT::ConditionNode
{
public:
    IsReached(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

protected:
    // Use tick() for ConditionNode
    virtual BT::NodeStatus tick() override;

private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg);
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    bool goal_reached_ = false;
    bool initialized_ = false;
    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point current;
    std::string state;
};

// Add RegisterNodes declaration
void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace GoTo
#endif