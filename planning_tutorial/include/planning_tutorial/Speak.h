#ifndef SPEAK_H
#define SPEAK_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "audio_common_msgs/action/tts.hpp"

namespace Speak {

using BT::NodeStatus;

class Speak : public BT::StatefulActionNode
{
public:
    Speak(const std::string& name, const BT::NodeConfig& config);

    // 필수 오버라이드 메소드
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    using TTS = audio_common_msgs::action::TTS;
    using GoalHandleTTS = rclcpp_action::ClientGoalHandle<TTS>;

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<TTS>::SharedPtr client_;

    // goal handle, future 저장
    rclcpp_action::ClientGoalHandle<TTS>::SharedPtr goal_handle_;
    std::shared_future<GoalHandleTTS::SharedPtr> future_goal_handle_;
    std::shared_future<typename GoalHandleTTS::WrappedResult> future_result_;

    bool finished_{false};
    bool success_{false};
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

} // namespace Speak

#endif
