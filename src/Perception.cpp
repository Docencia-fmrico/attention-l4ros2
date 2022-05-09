#include "Perception.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"


using namespace std::literals::chrono_literals;


namespace perception {

  Perception::Perception() : rclcpp_lifecycle::LifecycleNode("Perception")
  { 
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
    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());


    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Perception::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

    auto robot_node = ros2_knowledge_graph::new_node("tiago", "robot");
    graph_->update_node(robot_node);

    auto world_node = ros2_knowledge_graph::new_node("world", "world_origin");
    graph_->update_node(world_node);

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
    
    RCLCPP_INFO(get_logger(), "%d nodes", graph_->get_num_nodes());
    
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

      //Publish edge

      geometry_msgs::msg::PoseStamped pose_stamped_msg;
      pose_stamped_msg.pose = entity_state.pose;
      pose_stamped_msg.header.frame_id = "world";
      pose_stamped_msg.header.stamp = get_clock()->now();

      auto edge_content = ros2_knowledge_graph::new_content<geometry_msgs::msg::PoseStamped>(pose_stamped_msg, true);
      auto edge_world_obj = ros2_knowledge_graph::new_edge("world", "tiago", edge_content, true);

      edge_world_obj.content.type = ros2_knowledge_graph_msgs::msg::Content::POSE;
      edge_world_obj.content.pose_value = pose_stamped_msg;

      graph_->update_edge(edge_world_obj,true);

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /gazebo/get_entity_state");
    }

    return;
  }


  void Perception::links_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
  {  
    
    if(graph_ == nullptr){
      RCLCPP_INFO(get_logger(), "graph is null");
      return;
    }

    char long_name[256];
    std::vector<std::string> types = this->get_parameter("types").as_string_array();

    for (int j = 0; j < types.size();j++){
      std::string type_str = types.at(j);
      declare_parameter(type_str.c_str());
    }

    for (int i = 0; i < (msg->name).size() ; i++) {

      strcpy (long_name, msg->name[i].c_str());
      strtok(long_name, ":");
      std::string name(long_name);


      for (int k = 0; k < types.size();k++){
        if (name.find(types.at(k)) != std::string::npos){

          auto node_to_add = ros2_knowledge_graph::new_node(name, this->get_parameter(types.at(k).c_str()).as_string());
          graph_->update_node(node_to_add,false);

          geometry_msgs::msg::PoseStamped pose_stamped_msg;
          pose_stamped_msg.pose = msg->pose[i];
          pose_stamped_msg.header.frame_id = "world";
          pose_stamped_msg.header.stamp = get_clock()->now();

          auto edge_content = ros2_knowledge_graph::new_content<geometry_msgs::msg::PoseStamped>(pose_stamped_msg, false);
          auto edge_world_obj = ros2_knowledge_graph::new_edge("world", node_to_add.node_name, edge_content, false);

          edge_world_obj.content.type = ros2_knowledge_graph_msgs::msg::Content::POSE;
          edge_world_obj.content.pose_value = pose_stamped_msg;

          graph_->update_edge(edge_world_obj, false);
          std::cout << "node added"<< std::endl;
        }
      }
      
    }
    /*
    if (name.find("tiago") != std::string::npos){
      node_to_add.node_class = "robot";
    }
    */
    states_sub_ = NULL;
  }

}   //namespace perception