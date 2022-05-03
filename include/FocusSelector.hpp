// Copyright 2022 L4ROS2

#ifndef FOCUS_SELECTOR_HPP_
#define FOCUS_SELECTOR_HPP_

#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

//#include "ros2_knowledge_graph/GraphNode.hpp"


namespace focus_selector {

class FocusSelector : public rclcpp_lifecycle::LifecycleNode
{
public:
    FocusSelector();
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_configure(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state);

    void do_work();

private:
    //std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;

};

}   // namespace focus_selector

#endif  // FOCUS_SELECTOR_HPP_