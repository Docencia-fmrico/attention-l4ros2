// Copyright 2022 L4ROS2

#include "JointController.hpp"

namespace joint_controller
{


  JointController::JointController()
  : rclcpp_lifecycle::LifecycleNode("JointController")
  {
    declare_parameter("speed", 1.0);
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/joint_trajectory", 100);
    joints_names_.push_back("head_1_joint");
    joints_names_.push_back("head_2_joint");
    
  }

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
   
    speed_ = get_parameter("speed").get_value<double>();
   
    return CallbackReturnT::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_activate();
    
    return CallbackReturnT::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  
  void JointController::do_work() 
  { 
    RCLCPP_INFO(get_logger(), "DO WORK");
    
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_n;
    trajectory_msgs::msg::JointTrajectoryPoint right;
    right.positions = {1.56,0,};
    right.velocities = {speed_,0,};
    right.accelerations = {0.3,0,};
    right.effort = {1.0,0,};

    points_n.push_back(right);
    
    if (pub_->is_activated()) {
      trajectory_msgs::msg::JointTrajectory msg;

      msg.header.stamp = this->now();
      msg.points.resize(1);
      msg.joint_names.resize(1);
      
      msg.joint_names[0] = joints_names_[0];
      msg.points[0] = points_n[0];

      pub_->publish(msg);
    }
    
    return;
  }

}  // namespace joint_controller