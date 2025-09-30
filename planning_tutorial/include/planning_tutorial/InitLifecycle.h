#ifndef INIT_LIFECYCLE_H
#define INIT_LIFECYCLE_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

namespace LifecycleBT {

using BT::NodeStatus;

class InitLifecycle : public BT::SyncActionNode {
public:
    InitLifecycle(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    std::string current_Action;
    std::string prev_Action;

    std::vector<std::string> service_names_;
    std::vector<rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> clients_; 
    // std::vector<rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> deactivate_clients_;
    
    

    std::string config_file;
    std::vector<std::string> lifes;
    // std::vector<std::string> prev_lifecycle_nodes;

    bool initialized_ = false;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace LifecycleBT

#endif
