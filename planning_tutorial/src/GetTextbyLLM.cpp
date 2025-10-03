#include "planning_tutorial/GetTextbyLLM.h"
#include <chrono>

namespace GetTextbyLLM {

GetTextbyLLM::GetTextbyLLM(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    client_ = node_->create_client<rby1_custom_interfaces::srv::SetCaptionMode>("/set_caption_mode");
    pub_ = node_->create_publisher<std_msgs::msg::String>("/vlm/text_prompt", 10);
}

BT::PortsList GetTextbyLLM::providedPorts(){
    return 
    { 
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
        BT::BidirectionalPort<std::string>("heard_text", "Generated text from LLM"),
        BT::InputPort<std::string>("prompt", "Prompt to send to LLM"),
    };
}

BT::NodeStatus GetTextbyLLM::onStart() 
{
    finished_ = false;
    success_ = false;
    text.clear();
    textbyLLM.clear();

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service /set_caption_mode not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<rby1_custom_interfaces::srv::SetCaptionMode::Request>();
    request->mode = 4;

    // future_를 멤버가 아니라 로컬 변수로 사용
    auto future = client_->async_send_request(
        request,
        std::bind(&GetTextbyLLM::handleResponse, this, std::placeholders::_1));

    text = getInput<std::string>("heard_text").value();
    prompt = getInput<std::string>("prompt").value();
    RCLCPP_INFO(node_->get_logger(), "GetTextbyLLM: text: %s", text.c_str());
    RCLCPP_INFO(node_->get_logger(), "GetTextbyLLM: prompt: %s", prompt.c_str());


    auto msg = std_msgs::msg::String();
    msg.data = text + " " + prompt;
    pub_->publish(msg);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetTextbyLLM::onRunning() 
{
    if (!finished_) {
        return BT::NodeStatus::RUNNING;
    }

    if (success_){
        RCLCPP_INFO(node_->get_logger(), "GetTextbyLLM textbyLLM : %s", textbyLLM.c_str());
        setOutput("heard_text", textbyLLM);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void GetTextbyLLM::onHalted() 
{
    RCLCPP_WARN(node_->get_logger(), "GetTextbyLLM HALTED");
    finished_ = false;
    success_ = false;
    text.clear();
}

void GetTextbyLLM::handleResponse(
    std::shared_future<std::shared_ptr<rby1_custom_interfaces::srv::SetCaptionMode::Response>> future)
{
    auto response = future.get();
    success_ = response->success;
    textbyLLM = response->message;
    finished_ = true;
}


void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<GetTextbyLLM>("GetTextbyLLM");
}

}