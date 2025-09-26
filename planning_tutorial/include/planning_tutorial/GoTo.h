//GoTo.h

#ifnedf SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rby1_custom_interfaces/srv/CallLoc.hpp"

namespace GoTo {

using BT::NodeStatus;

class SetGoal : public BT::StatefulActionNode{
public:
    SetGoal(const std::string& name,const BT::NodeConfig& config);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<rby1_custom_interfaces::srv::CallLoc>::SharedPtr rb_client_;
    rclcpp::Client<rby1_custom_interfaces::srv::CallLoc>::SharedPtr rb_future_;
    
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future_;

    State current_state_;
};

class IsReached : public BT::SyncActionNode{
public:
    IsReached(const std::string& name, const BT::NodeConfig& config);

    NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class Speak : public BT::SyncActionNode {
public:
    Speak(const std::string& name, const BT::NodeConfig& config);
    NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}
#endif