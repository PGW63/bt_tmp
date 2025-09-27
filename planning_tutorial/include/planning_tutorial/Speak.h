// Speak.h

#ifndef SPEAK_H
#define SPEAK_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "autio_common_msgs/action/tts.hpp"


namespace Speak {

using BT::NodeStatus;

class Speak : public BT::StatefulActionNode
{
public:
    Speak(const std::string& name, const BT::NodeConfig& config);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    using TTS = audio_common_msgs::action::TTS;
    using GoalHandleTTS = rclcpp_action::ClientGoalHandle<TTS>;

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<TTS>::SharedPtr client_;

    std::shared_future<GoalHandleTTS::SharedPtr> future_result_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}
#endif