#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "planning_tutorial/FindObjects.h" 
#include "planning_tutorial/InitLifecycle.h"   // ◀ 새로 추가
#include "planning_tutorial/GoTo.h"
#include "planning_tutorial/Speak.h"
#include "planning_tutorial/Describe_person.h"
#include "planning_tutorial/Listen.h"
#include "planning_tutorial/GetTextbyLLM.h"

#include <chrono> // std::chrono_literals 사용
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_main_node");

    auto black_board = BT::Blackboard::create();
    black_board->set("node", node);
    black_board->set("lifecycle", "None");
    black_board->set("location", "None");
    black_board->set("willfinds", "None");
    black_board->set("text", "None");
    black_board->set("prompt", "None");
    black_board->set("heard_text", "None");

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<LifecycleBT::InitLifecycle>("InitLifecycle");
    factory.registerNodeType<Speak::Speak>("Speak");
    factory.registerNodeType<Listen::Listen>("Listen");
    factory.registerNodeType<GetTextbyLLM::GetTextbyLLM>("GetTextbyLLM");

    GoTo::RegisterNodes(factory);
    FindObjects::RegisterNodes(factory);
    Describe_person::RegisterNodes(factory);

    std::string package_name = ament_index_cpp::get_package_share_directory("planning_tutorial");
    std::string dir_name = "/bt_xmls";
    
    factory.registerBehaviorTreeFromFile(package_name+dir_name+"/GoTo.xml");
    factory.registerBehaviorTreeFromFile(package_name+dir_name+"/FindObjects.xml");
    factory.registerBehaviorTreeFromFile(package_name+dir_name+"/Describe_person.xml");
    factory.registerBehaviorTreeFromFile(package_name+dir_name+"/AskFavorite.xml");

    factory.registerBehaviorTreeFromFile(package_name+dir_name+"/dummy_main.xml");
    
    auto main_tree = factory.createTree("dummy_main", black_board);

    RCLCPP_INFO(node->get_logger(), "Behavior Tree started!");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Rate loop_rate(10); // 10Hz

    while (rclcpp::ok()) {
        
        main_tree.tickOnce(); 

        // ROS 이벤트(콜백 등) 처리
        executor.spin_some();
        
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(),
        "Behavior Tree finished ");

    rclcpp::shutdown();
    return 0;
}
