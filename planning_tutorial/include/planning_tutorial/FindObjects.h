// FindObjects.h

#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include <atomic> // ◀ IsDetected를 위해 필요

namespace FindObjects {

using BT::NodeStatus;

// ◀ [수정] SetObjects: SyncActionNode -> StatefulActionNode
class SetObjects : public BT::StatefulActionNode { 
public:
    SetObjects(const std::string& name, const BT::NodeConfig& config);

    // ◀ [수정] StatefulActionNode 메서드
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    // ◀ [수정] SetObjects 내부 상태
    enum class State {
        SERVICE_CALL,
        WAITING_FOR_RESPONSE
    };
    State current_state_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    // ◀ [수정] 서비스 클라이언트 및 Future
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future_;
};

// --- IsDetected 는 동일 ---
class IsDetected : public BT::StatefulActionNode {
public:
    IsDetected(const std::string& name, const BT::NodeConfig& config);
    
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();
private:
    enum class State {
        SERVICE_CALL,
        WAITING_FOR_RESPONSE,
        WAITING_FOR_TOPIC
    };

    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
    
    std::atomic<NodeStatus> atomic_status_; 
    State current_state_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future_;
};

// --- Pan, Speak 는 동일 ---
class Pan : public BT::SyncActionNode {
// ... (이하 동일)
public:
    Pan(const std::string& name, const BT::NodeConfig& config);
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

} // namespace FindObjects
#endif