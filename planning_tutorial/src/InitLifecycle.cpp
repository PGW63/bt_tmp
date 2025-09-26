#include "planning_tutorial/InitLifecycle.h"

namespace LifecycleBT {

InitLifecycle::InitLifecycle(const std::string& name, const BT::NodeConfig& config)
    : StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Configure할 lifecycle 노드들의 서비스 이름
    std::vector<std::string> services = {
        "/yolo_node/change_state",
        // 필요한 노드 추가
    };

    for (const auto& srv : services) {
        Target t;
        t.name = srv;
        t.client = node_->create_client<lifecycle_msgs::srv::ChangeState>(srv);
        targets_.push_back(t);
    }
}

BT::PortsList InitLifecycle::providedPorts() {
    return {};
}

NodeStatus InitLifecycle::onStart() {
    // 모든 서비스 준비 확인 후 Configure 요청
    for (auto& t : targets_) {
        if (!t.client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "Service not available: %s", t.name.c_str());
            return NodeStatus::FAILURE;
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        t.future = t.client->async_send_request(req);
    }
    return NodeStatus::RUNNING;
}

NodeStatus InitLifecycle::onRunning() {
    bool all_done = true;

    for (auto& t : targets_) {
        if (!t.configured) {
            if (t.future.valid() &&
                t.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                auto res = t.future.get();
                if (!res->success) {
                    RCLCPP_ERROR(node_->get_logger(), "Configure failed: %s", t.name.c_str());
                    return NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "Configured: %s", t.name.c_str());
                t.configured = true;
            } else {
                all_done = false;
            }
        }
    }

    if (all_done) {
        RCLCPP_INFO(node_->get_logger(), "All lifecycle nodes configured successfully!");
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

void InitLifecycle::onHalted() {
    // 필요시 Cleanup 처리 가능
}

void RegisterNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<InitLifecycle>("InitLifecycle");
}

} // namespace LifecycleBT
