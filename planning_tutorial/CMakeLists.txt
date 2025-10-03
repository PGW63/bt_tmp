cmake_minimum_required(VERSION 3.8)
project(planning_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rby1_custom_interfaces REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)



set(THIS_PACKAGE_DEPS
    behaviortree_cpp
    behaviortree_ros2
    std_msgs
    std_srvs
    lifecycle_msgs
    rby1_custom_interfaces
    audio_common_msgs
    rclcpp_action
    yaml-cpp    
)

install(DIRECTORY
  bt_xmls
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(bt_main_node src/bt_main.cpp
                       src/FindObjects.cpp
                       src/InitLifecycle.cpp
                       src/Speak.cpp
                       src/GoTo.cpp
                       src/Describe_person.cpp
                       src/Listen.cpp
                       src/GetTextbyLLM.cpp)

target_link_libraries(bt_main_node
  yaml-cpp
)
                       
ament_target_dependencies(bt_main_node
  rclcpp
  std_msgs
  lifecycle_msgs
  behaviortree_cpp
  rby1_custom_interfaces
  audio_common_msgs
  rclcpp_action
  yaml-cpp
  )

target_include_directories(bt_main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rby1_custom_interfaces_INCLUDE_DIRS}
)

install(TARGETS
  bt_main_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_xmls 
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY configs 
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
