#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "planning_tutorial/FindObjects.h" 
#include "planning_tutorial/InitLifecycle.h"   // ◀ 새로 추가

#include <chrono> // std::chrono_literals 사용

int main(int argc, char** argv)
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_main_node");

    // BehaviorTreeFactory 생성 및 사용자 노드 등록
    BT::BehaviorTreeFactory factory;
    FindObjects::RegisterNodes(factory);       // 기존 노드 등록
    LifecycleBT::RegisterNodes(factory);       // ◀ InitLifecycle 등록

    // 블랙보드 생성
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    // Output Port로 "willfinds" 초기값 설정
    blackboard->set("willfinds", std::string("pitch")); 

    // XML로부터 트리 생성
    BT::Tree tree = factory.createTreeFromFile(
        "/home/gw/plan_ws/src/bt_tmp/planning_tutorial/bt_xmls/findObjects.xml",
        blackboard);

    RCLCPP_INFO(node->get_logger(), "Behavior Tree started!");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Rate loop_rate(10); // 10Hz

    // --- 메인 루프 ---
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        
        status = tree.tickOnce(); 

        // ROS 이벤트(콜백 등) 처리
        executor.spin_some();
        
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(),
        "Behavior Tree finished with status: %s", BT::toStr(status).c_str());

    rclcpp::shutdown();
    return 0;
}
