#include "planning_tutorial/Describe_person.h"

namespace Describe_person {

// ---------------- Constructor ----------------
DescribePerson::DescribePerson(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      finished_(false),
      success_(false),
      text("")
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    client_ = node_->create_client<rby1_custom_interfaces::srv::SetCaptionMode>("/set_caption_mode");
}

// ---------------- Ports ----------------
BT::PortsList DescribePerson::providedPorts()
{
    return {
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
        BT::OutputPort<std::string>("text")
    };
}

// ---------------- onStart ----------------
BT::NodeStatus DescribePerson::onStart()
{
    finished_ = false;
    success_ = false;
    text.clear();

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service /set_caption_mode not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<rby1_custom_interfaces::srv::SetCaptionMode::Request>();
    request->mode = 3;

    // future_를 멤버가 아니라 로컬 변수로 사용
    auto future = client_->async_send_request(
        request,
        std::bind(&DescribePerson::handleResponse, this, std::placeholders::_1));

    return BT::NodeStatus::RUNNING;
}

// ---------------- onRunning ----------------
BT::NodeStatus DescribePerson::onRunning()
{
    if (!finished_) {
        return BT::NodeStatus::RUNNING;
    }

    setOutput("text", text);

    return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------- onHalted ----------------
void DescribePerson::onHalted()
{
    finished_ = true;
    success_ = false;
    text.clear();
}

// ---------------- handleResponse ----------------
void DescribePerson::handleResponse(
    std::shared_future<std::shared_ptr<rby1_custom_interfaces::srv::SetCaptionMode::Response>> future)
{
    auto response = future.get();
    success_ = response->success;
    text = response->message;
    finished_ = true;
}

// ---------------- RegisterNodes ----------------
void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<DescribePerson>("DescribePerson");
}

} // namespace Describe_person
