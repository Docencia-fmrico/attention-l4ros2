// Copyright 2022 L4ROS2

#include "JointController.hpp"
#include <math.h>
using namespace std::literals::chrono_literals;

namespace joint_controller
{


  JointController::JointController()
  : rclcpp_lifecycle::LifecycleNode("JointController")
  {
    declare_parameter("speed", 5.0);
    declare_parameter("robot_name", "tiago");
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/joint_trajectory", 100);
    joints_names_.push_back("head_1_joint");
    joints_names_.push_back("head_2_joint");
    
  }

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
   
    speed_ = get_parameter("speed").get_value<double>();
    robot_name_ = get_parameter("robot_name").get_value<std::string>();

    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());

    return CallbackReturnT::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_activate();
    
    return CallbackReturnT::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_deactivate();

    return CallbackReturnT::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  JointController::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

double getAngle(double y, double x){
  double ang;

  if (x == 0.0){
    ang = atan(INFINITY);
  }
  else{
    ang = atan(y/x);
  }

  if (x < 0){
    if(y < 0){
      ang += -M_PI; 
    }
    if(y > 0){
      ang += M_PI;
    }
  }
  return ang;
}

void getAngleBetween(geometry_msgs::msg::Pose & r_pos, geometry_msgs::msg::Pose & o_pos, double & yaw, double & pitch){

  r_pos.position.z = 1;
  
  geometry_msgs::msg::Pose diff;
  diff.position.x = o_pos.position.x - r_pos.position.x;
  diff.position.y = o_pos.position.x - r_pos.position.y;
  diff.position.z = o_pos.position.x - r_pos.position.z;

  // calc the arctan in XY -> yaw, and the arctan in XZ

  double ang_1 = getAngle(diff.position.y, diff.position.x);
  double ang_2 = getAngle(o_pos.position.y, o_pos.position.x);

  yaw =  ang_1 - ang_2;

  double ang_3 = getAngle(diff.position.z, diff.position.x);
  double ang_4 = getAngle(o_pos.position.z, o_pos.position.x);

  pitch = ang_3 - ang_4;
}
  
  void JointController::do_work() 
  { 
    //RCLCPP_INFO(get_logger(), "DO WORK");    
    
    // for every robot check its position and difference eiththe target

    RCLCPP_INFO(get_logger(), "before getting nodes");

    std::vector<ros2_knowledge_graph_msgs::msg::Node> node_list = graph_->get_nodes();
   
    std::map<std::string, ros2_knowledge_graph_msgs::msg::Edge> r_positions_edges;
    std::vector<std::string> r_names;

    
    std::vector<ros2_knowledge_graph_msgs::msg::Edge> world_edges;

    RCLCPP_INFO(get_logger(), "before getting eedges");

    world_edges = graph_->get_edges("world", robot_name_, ros2_knowledge_graph_msgs::msg::Content::POSE);
    //world_nodes.insert(world_nodes.end(), temp_nodes.begin(), temp_nodes.end() );

    if (world_edges.size() == 1){ // in this use, we'll only add 1 edges of position
      r_positions_edges[robot_name_] = world_edges.at(0);
    }
    else{
      RCLCPP_ERROR(get_logger(),"Couldn't get a reliable position source from %s", robot_name_.c_str());
      return;
    }
    
    // now get the target to see for the robot
    std::vector<ros2_knowledge_graph_msgs::msg::Edge> temp_edges;
    std::string object_name;

    for (auto & n : node_list){
      temp_edges = graph_->get_edges(robot_name_, n.node_name, ros2_knowledge_graph_msgs::msg::Content::STRING);

      if (temp_edges.size() == 1 && temp_edges.at(0).content.string_value == "target"){
        object_name = n.node_name;
      }
      else{
        RCLCPP_ERROR(get_logger(),"Couldn't get a reliable name for target");
        return;
      }
    }


    RCLCPP_INFO (get_logger(), "Robot %s targets %s", robot_name_.c_str(), object_name.c_str());
    geometry_msgs::msg::Pose r_pos, o_pos;

    r_pos = r_positions_edges[robot_name_].content.pose_value.pose;
    temp_edges = graph_->get_edges("world", object_name, ros2_knowledge_graph_msgs::msg::Content::POSE);
    
    if(temp_edges.size() == 1){
      o_pos = temp_edges.at(0).content.pose_value.pose;
    }
    else{
      RCLCPP_ERROR(get_logger(),"Couldn't get a reliable position source from %s", object_name.c_str());
    }
    double yaw, pitch;

    getAngleBetween(r_pos, o_pos, yaw, pitch);

    move_to_position(yaw, pitch);
    return;
  }


  void JointController::move_to_position(double yaw, double pitch){
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_n;
    trajectory_msgs::msg::JointTrajectoryPoint goal_position;
    
    goal_position.positions = {0.0,0.0};
    goal_position.velocities = {0.0,0.0};
    goal_position.accelerations = {0.0,0.0};
    goal_position.effort = {0.0,0.0};

    points_n.push_back(goal_position);
    
    if (pub_->is_activated()) {
      trajectory_msgs::msg::JointTrajectory msg;

      //msg.header.stamp = this->now();
      msg.points.resize(1);
      msg.joint_names.resize(2);
      
      msg.joint_names = joints_names_;
      msg.points = points_n;
      msg.points[0].positions[0] = yaw*(PI/180.0);
      msg.points[0].positions[1] = pitch*(PI/180.0);
      msg.points[0].velocities[0] = speed_;
      msg.points[0].velocities[1] = speed_;
      msg.points[0].accelerations[0] = 0.3;
      msg.points[0].accelerations[1] = 0.3;
      msg.points[0].effort[0] = 5.0;
      msg.points[0].effort[1] = 5.0;
      msg.points[0].time_from_start = rclcpp::Duration(1s);
      

      pub_->publish(msg);
    }
    return;
  }
}  // namespace joint_controller