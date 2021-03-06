// Copyright 2022 L4ROS2

#ifndef FOCUS_SELECTOR_HPP_
#define FOCUS_SELECTOR_HPP_

#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"


namespace focus_selector {

typedef std::vector<ros2_knowledge_graph_msgs::msg::Node> nodes_vector;

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
    void fake_watching_updater();

private:
    std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;

    const std::vector<ros2_knowledge_graph_msgs::msg::Node> get_nodes_by_class(const std::string & class_name);
    
    nodes_vector get_target_nodes_from_edges(std::vector<ros2_knowledge_graph_msgs::msg::Edge> edges);

    double timer = -1.0;
    std::map<std::string, double> init_times_;

    bool exist_edge(std::string source, std::string target, std::string content);

    void liberate_robot(ros2_knowledge_graph_msgs::msg::Node robot,
                                     std::vector<ros2_knowledge_graph_msgs::msg::Edge> target_edge,
                                     nodes_vector & free_robots);

};

}   // namespace focus_selector

#endif  // FOCUS_SELECTOR_HPP_