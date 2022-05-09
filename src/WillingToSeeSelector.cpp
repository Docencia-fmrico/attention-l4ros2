
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

double getDistanceBetween(std::string robot_name, std::string object_name, std::map<std::string, ros2_knowledge_graph_msgs::msg::Edge> map){

  geometry_msgs::msg::Pose r_pose = map[robot_name].content.pose_value.pose;
  geometry_msgs::msg::Pose o_pose = map[object_name].content.pose_value.pose;

  return sqrt(pow(r_pose.position.x - o_pose.position.x,2) + pow(r_pose.position.y - o_pose.position.y,2) + pow(r_pose.position.z - o_pose.position.z,2));

}

void WillingToSeeSelector::addWantToSeeEdge(std::string name1, std::string name2){

  std::string want_see_id = "want_to_see";

  auto edge_content = ros2_knowledge_graph::new_content<std::string>(want_see_id, true);
  auto edge_want_to_see = ros2_knowledge_graph::new_edge(name1, name2, edge_content, true);

  edge_want_to_see.content.type = ros2_knowledge_graph_msgs::msg::Content::STRING;
  edge_want_to_see.content.string_value = want_see_id;

  graph_->update_edge(edge_want_to_see, true);

  RCLCPP_INFO(get_logger(), "Want to see edge between %s and %s added", name1.c_str(), name2.c_str());

}

  void WillingToSeeSelector::do_work() 
  { 
    // get all the robots nodes
    
    std::vector<ros2_knowledge_graph_msgs::msg::Node> node_list = graph_->get_nodes();
   
    std::vector<std::string> robot_names;

    // get robot names

    for (auto & node : node_list){
      if( node.node_class == robot_class_name){
        robot_names.push_back(node.node_name);
         RCLCPP_INFO(get_logger(), "%s detected with name %s\n", node.node_class.c_str(), node.node_name.c_str());
      }
    }

    // get edges from world
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

    // get the distance between robots and the rest of interesting objects 
    double dist;
    std::string want_see_id = "want_to_see";
    for (auto & node : node_list){

        if(std::find(accepted_types_.begin(), accepted_types_.end(), node.node_class) != accepted_types_.end()){

          // it is an intersesting object
          // get the distance to all the robots

          for (auto & r_name : robot_names){
            temp_edges = graph_-> get_edges(r_name, node.node_name, ros2_knowledge_graph_msgs::msg::Content::STRING);
            dist  = getDistanceBetween(r_name, node.node_name, world_edges_map);

            if (dist < ATTENTION_RADIUS){
              addWantToSeeEdge(r_name, node.node_name);
            }else if(temp_edges.size() > 0){
              for (auto & edge : temp_edges){
                if(edge.content.string_value == want_see_id){
                  graph_->remove_edge(edge, true);
                }
              }
            }
          }

        }


    }

    return;
  }

}   //namespace WillingToSeeSelector