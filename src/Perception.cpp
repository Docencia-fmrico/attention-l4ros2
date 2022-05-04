#include "Perception.hpp"

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
    
    //radius_ = get_parameter("radius").get_value<double>();
   
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
    //TODO: if we want to publish the positions we may have to delete this line to update them.

  }

}   //namespace perception