// Copyright 2022 L4ROS2

#include "JointController.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (argc != 3) return -1;
  double yaw = atof(argv[1]);
  double pitch = atof(argv[2]);

  auto node = std::make_shared<joint_controller::JointController>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  //rclcpp::spin(node->get_node_base_interface());
  rclcpp::Rate rate(5);
  
  while (rclcpp::ok()) {
    node->move_to_position(yaw,pitch);

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}