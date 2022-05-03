#include "Perception.hpp"


namespace perception {

  Perception::Perception() : rclcpp_lifecycle::LifecycleNode("Perception")
  {
    declare_parameter("speed", 5.0);
    preception_client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("perception");

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
   
    radius_ = get_parameter("radius").get_value<double>();
   
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Perception::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    //pub_->on_activate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Perception::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    //pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Perception::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    //pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  void Perception::do_work() 
  { 
    //link_names_
    RCLCPP_INFO(get_logger(), "DO WORK");
      
    /*
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("perception_client");

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = argv[1];
    request->reference_frame = "tiago";

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    //*/


    //pub_->publish(msg);
    
    return;
  }






  void Perception::links_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
  {
    for (int i = 0; i < (msg->name).size(); i++) {
      //TO DO: filter with a black list.
      std::cout << msg->name[i] << " was introduced in the knowledge base." << std::endl;
      //TO DO: add to the knowledge base every gazebo object name if not in blacklist.
    }
    link_names_ = msg->name;
    
    states_sub_ = NULL; //destroy the subscriber to stop getting messages.

    /*
    for (int i = 0; i < (msg->name).size(); i++) {
      /*
      if (endsWith(msg->name[i], "::body") &&
          getDistance(msg->pose[i], tiago_base_link_pose_) < ATTENTION_RADIUS) {
        std::cout << msg->name[i] << " was introduced in the knowledge base." << std::endl;

        //TO DO: introduce in the knowledge base the poses (and link names?) whose
        //names ends with "::body" (endsWith() function) and whose distance to the
        //tiago_base_link_pose_ is less than ATTENTION_RADIUS.

        //TO DO: the pose of the object will be saved making use of a service (ros2
        //service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState
        //'{name: 'PotatoChipChair_3::PotatoChipChair::body', reference_frame: 'tiago'}').
      }
      link_names_.push_back(msg->name[i]);
      std::cout << msg->name[i] << " was introduced in the knowledge base." << std::endl;

    }
    //msg->name;
    //msg->pose;
    //msg->twist;
    //*/
  }

}   //namespace perception