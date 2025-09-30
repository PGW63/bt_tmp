#ifndef PLANNING_TUTORIAL_DESCRIBE_PERSON_H
#define PLANNING_TUTORIAL_DESCRIBE_PERSON_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rby1_custom_interfaces/srv/set_caption_mode.hpp"
#include <atomic>
#include <memory>
#include <string>

namespace Describe_person {

using BT::NodeStatus;

class DescribePerson : public BT::StatefulActionNode
{
public:
    DescribePerson(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<rby1_custom_interfaces::srv::SetCaptionMode>::SharedPtr client_;

    std::atomic_bool finished_;
    bool success_;
    std::string text;

    void handleResponse(
        std::shared_future<std::shared_ptr<rby1_custom_interfaces::srv::SetCaptionMode::Response>> future);
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace Describe_person

#endif
