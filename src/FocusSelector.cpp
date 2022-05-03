// Copyright 2022 L4ROS2

#include "FocusSelector.hpp"


namespace focus_selector {

  FocusSelector::FocusSelector() : rclcpp_lifecycle::LifecycleNode("FocusSelector")
  { 
    //graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>("FocusSelector");
  }

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT FocusSelector::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
   
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FocusSelector::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FocusSelector::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FocusSelector::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  void FocusSelector::do_work() 
  { 
    RCLCPP_INFO(get_logger(), "DO WORK");
      
    
    
    return;
  }

}   //namespace focus_selector