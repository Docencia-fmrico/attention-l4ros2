

#include "JointController.hpp"

namespace joint_controller
{


  JointController::JointController()
  : rclcpp_lifecycle::LifecycleNode("JointController")
  {
    declare_parameter("speed", 0.34);
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_pub", 100);
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

    if (pub_->is_activated()) {
      trajectory_msgs::msg::JointTrajectory msg;
      msg.joint_names = joints_names_;

      pub_->publish(msg);
    }
    
  }

}  // namespace joint_controller