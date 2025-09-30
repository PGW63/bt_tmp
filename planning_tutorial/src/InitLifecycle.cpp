#include "planning_tutorial/InitLifecycle.h"
#include "yaml-cpp/yaml.h"

namespace LifecycleBT {

InitLifecycle::InitLifecycle(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    config_file = ament_index_cpp::get_package_share_directory("planning_tutorial") + "/configs/lifecycle_node.yaml";
}

BT::PortsList InitLifecycle::providedPorts() {
    return 
    {
        BT::InputPort<rclcpp::Node::SharedPtr>("node"),
        BT::InputPort<std::string>("lifecycle", "what's lifecycle nodes will be turned on")
    };
}

BT::NodeStatus InitLifecycle::tick(){
    current_Action = getInput<std::string>("lifecycle").value();
    RCLCPP_INFO(node_->get_logger(), "InitLifecycle current_Action = %s", current_Action.c_str());

    if(!initialized_)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(config_file);
            YAML::Node lifenodes = config["lifecycle"];
            if(!lifenodes){
                RCLCPP_ERROR(node_->get_logger(), "No lifecycle nodes in config file");
                return BT::NodeStatus::FAILURE;
            }

            for (const auto& life : lifenodes){
                if(life["name"].as<std::string>() == current_Action){
                    for(const auto& l : life["lifecycle_nodes"] ){
                        service_names_.push_back((l.as<std::string>()+"/change_state").c_str());
                        RCLCPP_INFO(node_->get_logger(), "will configure --> %s", (l.as<std::string>()).c_str());
                    }
                    break;
                }
            }
        } catch(const std::exception& e){
            RCLCPP_ERROR(node_->get_logger(), "Failed to load config file: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }



        for (const auto& srv : service_names_) {
            clients_.push_back(node_->create_client<lifecycle_msgs::srv::ChangeState>(srv));
        }

        initialized_ = true;
    }

    for (size_t i = 0; i < clients_.size(); i++){
        auto& client = clients_[i];
        const auto& srv_name = service_names_[i];

        if (!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_ERROR(node_->get_logger(), "Service not avaiable: %s", srv_name.c_str());
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

        auto future = client->async_send_request(request);
        // auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));

        // if (result != rclcpp::FutureReturnCode::SUCCESS) {
        //     RCLCPP_ERROR(node_->get_logger(), "Failed to call service: %s", srv_name.c_str());
        //     return NodeStatus::FAILURE;
        // }

        // auto res = future.get();
        // if (!res->success) {
        //     RCLCPP_ERROR(node_->get_logger(), "Failed to configure node: %s", srv_name.c_str());
        //     return NodeStatus::FAILURE;
        // }

        RCLCPP_INFO(node_->get_logger(), "Configured node: %s", srv_name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "All nodes configured successfully.");
    return NodeStatus::SUCCESS;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<InitLifecycle>("InitLifecycle");
}

} // namespace LifecycleBT
