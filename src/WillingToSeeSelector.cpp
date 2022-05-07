
#include "WillingToSeeSelector.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

namespace willing_to_see {

  WillingToSeeSelector::WillingToSeeSelector() : rclcpp_lifecycle::LifecycleNode("WillingToSeeSelector")
  { 
   
  }

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT WillingToSeeSelector::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
    
    radius_ = ATTENTION_RADIUS; // get_parameter("radius").get_value<double>();
    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT WillingToSeeSelector::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    //pub_->on_activate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT WillingToSeeSelector::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    //pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT WillingToSeeSelector::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    //pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  void WillingToSeeSelector::do_work() 
  { 
    // get all the robots nodes
    
    std::vector<ros2_knowledge_graph_msgs::msg::Node> node_list = graph_->get_nodes();
   
    std::vector<std::string> robot_names;

    for (auto & node : node_list){
      if( node.node_class == robot_class_name){
        robot_names.push_back(node.node_name);
         RCLCPP_INFO(get_logger(), "%s detected with name %s\n", node.node_class.c_str(), node.node_name.c_str());
      }
    }

    std::vector<ros2_knowledge_graph_msgs::msg::Edge> world_nodes = graph_->get_edges_from_node_by_data("world", ""); // try to get all the edges

    for (auto & edge : world_nodes){

     RCLCPP_INFO(get_logger(), "%s connected to %s by a %s edge", edge.source_node_id.c_str(), edge.target_node_id.c_str(), ros2_knowledge_graph::to_string(edge.content.type));
    }


    return;
  }

}   //namespace WillingToSeeSelector