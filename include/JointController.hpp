// Copyright 2022 L4ROS2

#ifndef JOINT_CONTROLLER_HPP_
#define JOINT_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include <tf2/LinearMath/Quaternion.h>
//#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <math.h>

#define PI 3.141592653589793238463

namespace joint_controller
{

class JointController : public rclcpp_lifecycle::LifecycleNode
{
public:
    JointController();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_configure(const rclcpp_lifecycle::State & state);

    void do_work();

    void move_to_position(double yaw, double pitch);
    
private:
    double speed_;
    std::string  robot_name_;
    std::vector<std::string> joints_names_;
    std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
    rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

}   // namespace joint_controller

#endif  // JOINT_CONTROLLER_HPP_