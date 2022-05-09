// Copyright 2022 L4ROS2

#include "FocusSelector.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<focus_selector::FocusSelector>();
  
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    node->do_work();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}