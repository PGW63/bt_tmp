#ifndef INIT_LIFECYCLE_H
#define INIT_LIFECYCLE_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace LifecycleBT {

using BT::NodeStatus;

class InitLifecycle : public BT::StatefulActionNode {
public:
    InitLifecycle(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class Stage { CONFIGURE, ACTIVATE, DONE };
    Stage current_stage_;

    rclcpp::Node::SharedPtr node_;

    struct Target {
        std::string name; // 예: "/yolo_node"
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future;
        bool configured = false;
        bool activated = false;
    };

    std::vector<Target> targets_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace LifecycleBT

#endif
