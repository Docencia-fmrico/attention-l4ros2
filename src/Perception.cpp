#include "Perception.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"
using namespace std::literals::chrono_literals;


namespace perception {

  Perception::Perception() : rclcpp_lifecycle::LifecycleNode("Perception")
  { 
    //graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>("Perception");
    perception_client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
    this->declare_parameter("types");
    using namespace std::placeholders;
    states_sub_ = create_subscription<gazebo_msgs::msg::LinkStates>(
      "/gazebo/link_states",
      10,
      std::bind(&Perception::links_callback, this, _1));

  }

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT Perception::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
    radius_ = ATTENTION_RADIUS; // get_parameter("radius").get_value<double>();
    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
    auto world_node = ros2_knowledge_graph::new_node("world", "world_origin");
    graph_->update_node(world_node);
    //graph_ = new ros2_knowledge_graph::GraphNode(shared_from_this());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Perception::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Perception::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Perception::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  void Perception::do_work() 
  { 
    //link_names_
    
    RCLCPP_INFO(get_logger(), "%d nodes", graph_->get_num_nodes());

    std::vector<ros2_knowledge_graph_msgs::msg::Edge> edges_list = graph_->get_edges("world", "tiago::head_2_link", ros2_knowledge_graph_msgs::msg::Content::POSE);
    
    if (edges_list.empty()){
      RCLCPP_INFO(get_logger(), "poses edges from world to tiago is empty");
    }
    else{
      RCLCPP_INFO(get_logger(), "POSES EDGES:");
      for (auto & e : edges_list){

          RCLCPP_INFO(get_logger(),"source %s | target %s | type %d", e.source_node_id.c_str(), e.target_node_id.c_str(), e.content.type);


      }
    }
    
    std::vector<ros2_knowledge_graph_msgs::msg::Edge> error_edges_list = graph_->get_edges("world", "tiago::head_2_link", ros2_knowledge_graph_msgs::msg::Content::ERROR);

    if (error_edges_list.empty()){
      RCLCPP_INFO(get_logger(), "error edges from world to tiago is empty");
    }else{
      RCLCPP_INFO(get_logger(), "ERROR EDGES:");
      for (auto & e : error_edges_list){

          RCLCPP_INFO(get_logger(),"source %s | target %s | type %d", e.source_node_id.c_str(), e.target_node_id.c_str(), e.content.type);


      }
    }
    
    /*
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("perception_client");
  {
    RCLCPP_INFO(get_logger(), "DO WORK");


    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "tiago";
    request->reference_frame = "world";

    while (!perception_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = perception_client_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      gazebo_msgs::msg::EntityState entity_state = result.get()->state;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tiago x: %f, y: %f", entity_state.pose.position.x, entity_state.pose.position.y);

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    return;
  }


  void Perception::links_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
  {  
    //RCLCPP_INFO(get_logger(), "links_callback");
    if(!link_names_.empty()) return;

    if(graph_ == nullptr){
      RCLCPP_INFO(get_logger(), "graph is null");
      return;
    }
    
    

    char long_name[256];

    RCLCPP_INFO(this->get_logger(), "callback link_state");

    for (int i = 0; i < (msg->name).size() ; i++) {
      //TO DO: filter with a black list.
      RCLCPP_INFO(get_logger(), "it %d\n", i);
=======
    std::vector<std::string> types = this->get_parameter("types").as_string_array();

    for (int j = 0; j < types.size();j++){
      std::string type_str = types.at(j);
      declare_parameter(type_str.c_str());
    }

    for (int i = 0; i < (msg->name).size() ; i++) {

      strcpy (long_name, msg->name[i].c_str());
      //strtok(long_name, ":");
      std::string name(long_name);

      RCLCPP_INFO(get_logger(), "name %s created", name.c_str());

      auto node_to_add = ros2_knowledge_graph::new_node(name, "object");
      //node_to_add.node_class = "object";
      /*
      if (name.find("Female") != std::string::npos || name.find("Male") != std::string::npos) {
        name = "Person::"+ name;
        std::cout << name << " was introduced in the knowledge base." << std::endl;
        node_to_add.node_class = "person";
      }
      */

      if (name.find("tiago") != std::string::npos){
        node_to_add.node_class = "robot";
      }

      //node_to_add.node_name = name;
      RCLCPP_INFO(get_logger(), "node_to_add filled");

      graph_->update_node(node_to_add);
      geometry_msgs::msg::PoseStamped pose_stamped_msg;
      pose_stamped_msg.pose = msg->pose[i];
      pose_stamped_msg.header.frame_id = "world";
      pose_stamped_msg.header.stamp = get_clock()->now();

      auto edge_content = ros2_knowledge_graph::new_content<geometry_msgs::msg::PoseStamped>(pose_stamped_msg, true);

      RCLCPP_INFO(get_logger(), "type content %d | error type %d", edge_content.type, ros2_knowledge_graph_msgs::msg::Content::ERROR);

      auto edge_world_obj = ros2_knowledge_graph::new_edge("world", node_to_add.node_name, edge_content, true);

      edge_world_obj.content.type = ros2_knowledge_graph_msgs::msg::Content::POSE;
      edge_world_obj.content.pose_value = pose_stamped_msg;

      graph_->update_edge(edge_world_obj, true);

      std::cout << "node added"<< std::endl;
      //TO DO: add to the knowledge base every gazebo object name if not in blacklist.
    }
    link_names_ = msg->name;
    
    //states_sub_ = NULL; //destroy the subscriber to stop getting messages.


      for (int n = 0; n < types.size();n++){
        if (name.find(types.at(n)) != std::string::npos) {
          name = this->get_parameter(types.at(n).c_str()).as_string() + name;
          std::cout << name << " was introduced in the knowledge base with pose: (" << msg->pose[i].position.x << ", " << msg->pose[i].position.y << ")." << std::endl;
          
        }
      }
      
      //TO DO: add to the knowledge base every gazebo object name if it is in the dictionary.
    }
    link_names_ = msg->name;
    
    states_sub_ = NULL; //destroy the subscriber to stop getting messages.

  }

}   //namespace perception