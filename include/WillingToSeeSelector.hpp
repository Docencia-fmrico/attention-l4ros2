
// Copyright 2022 L4ROS2

#ifndef WillingToSeeSelector_HPP_nullptr
#define WillingToSeeSelector_HPP_

#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"


#define ATTENTION_RADIUS 5.0

namespace willing_to_see {

class WillingToSeeSelector : public rclcpp_lifecycle::LifecycleNode
{
public:
    WillingToSeeSelector();
    
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
    void addWantToSeeEdge(std::string name1, std::string name2);

    double radius_;

    std::map<std::string, geometry_msgs::msg::Pose> locations_;
    std::string robot_class_name = "robot";
    const std::vector<std::string> accepted_types_ = {"Person", "Chair"};
    std::vector<std::string> link_names_;
    std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
    //geometry_msgs::msg::Pose current_pos_;

};

}   // namespace WillingToSeeSelector

#endif  // WillingToSeeSelector_HPP_