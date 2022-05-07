#include "WillingToSeeSelector.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<willing_to_see::WillingToSeeSelector>();
 
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  //node->configure();
  //node->activate();
  //rclcpp::spin(node->get_node_base_interface());

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}