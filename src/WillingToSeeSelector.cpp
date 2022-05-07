
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

    std::vector<ros2_knowledge_graph_msgs::msg::Edge> world_nodes, temp_edges;
    std::map<std::string, ros2_knowledge_graph_msgs::msg::Edge> world_edges_map; // so we can access the Edge given the name
    //RCLCPP_INFO(get_logger(), "world got %d edges", world_nodes.size());
    for (auto & node : node_list){
      temp_edges = graph_->get_edges("world", node.node_name, ros2_knowledge_graph_msgs::msg::Content::POSE);
      //world_nodes.insert(world_nodes.end(), temp_nodes.begin(), temp_nodes.end() );

      if (temp_edges.size() == 1){ // in this use, we'll only add 1 edges of position
        world_edges_map[node.node_name] = temp_edges.at(0);
      }
    }

    RCLCPP_INFO(get_logger(), "Edges connected to world %d", world_edges_map.size()); 


    return;
  }

}   //namespace WillingToSeeSelector