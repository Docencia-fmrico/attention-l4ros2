// Copyright 2022 L4ROS2

#include <iostream>
#include <cstring>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "FocusSelector.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#define SECONDS_TO_STARE 2
#define SECONDS_TAKEN_TO_WATCH 5

using ros2_knowledge_graph::GraphNode;
using ros2_knowledge_graph::new_node;
using ros2_knowledge_graph::new_edge;
using ros2_knowledge_graph::get_property;
using ros2_knowledge_graph::add_property;


namespace focus_selector {

  FocusSelector::FocusSelector() : rclcpp_lifecycle::LifecycleNode("FocusSelector")
  {}

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT FocusSelector::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
    
    graph_ = std::make_shared<GraphNode>(shared_from_this());
   
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FocusSelector::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

    graph_->update_node(new_node("world", "World"));


    auto tmp_node = new_node("object_1", "Object");
    //add_property(tmp_node, "ros_time", shared_from_this()->now().seconds());
    //add_property(tmp_node, "available", true);
    graph_->update_node(tmp_node);

    tmp_node = new_node("object_2", "Object");
    //add_property(tmp_node, "ros_time", shared_from_this()->now().seconds());
    //add_property(tmp_node, "available", true);
    graph_->update_node(tmp_node);
    
    tmp_node = new_node("object_3", "Object");
    //add_property(tmp_node, "ros_time", shared_from_this()->now().seconds());
    //add_property(tmp_node, "available", true);
    graph_->update_node(tmp_node);

    tmp_node = new_node("object_4", "Object");
    //add_property(tmp_node, "ros_time", shared_from_this()->now().seconds());
    //add_property(tmp_node, "available", true);
    graph_->update_node(tmp_node);

    tmp_node = new_node("robot_1", "Robot");
    //add_property(tmp_node, "count_started", false);
    graph_->update_node(tmp_node);
    graph_->update_edge(new_edge<std::string>("robot_1", "object_1", "want_to_see"));
    graph_->update_edge(new_edge<std::string>("robot_1", "object_2", "want_to_see"));
    graph_->update_edge(new_edge<std::string>("robot_1", "object_3", "want_to_see"));

    tmp_node = new_node("robot_2", "Robot");
    //add_property(tmp_node, "count_started", false);
    graph_->update_node(tmp_node);
    graph_->update_edge(new_edge<std::string>("robot_2", "object_3", "want_to_see"));
    graph_->update_edge(new_edge<std::string>("robot_2", "object_4", "want_to_see"));
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FocusSelector::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FocusSelector::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());

    return CallbackReturnT::SUCCESS;
  }
  
  nodes_vector FocusSelector::get_target_nodes_from_edges(std::vector<ros2_knowledge_graph_msgs::msg::Edge> edges) {
    nodes_vector result;

    for(auto edge : edges) {
      auto node = graph_->get_node(edge.target_node_id);
      result.push_back(node.value());
    }

    return result;
  }


  void FocusSelector::liberate_robot(ros2_knowledge_graph_msgs::msg::Node robot,
                                     std::vector<ros2_knowledge_graph_msgs::msg::Edge> target_edge,
                                     nodes_vector & free_robots) {
    RCLCPP_INFO(get_logger(), "liberating: %s", robot.node_name.c_str());

    add_property(robot, "count_started", false);
    graph_->update_node(robot);

    auto object_node = graph_->get_node(target_edge[0].target_node_id);
    add_property(object_node.value(), "ros_time", shared_from_this()->now().seconds());
    add_property(object_node.value(), "available", true);
    graph_->update_node(object_node.value());

    graph_->remove_edge(target_edge[0]);

    free_robots.push_back(robot);
  }

  void FocusSelector::do_work() 
  { 
    /* Getting all robot nodes. */
    const nodes_vector robots = get_nodes_by_class("Robot");

    /* Collecting all objects that need to be watched. */
    std::map<std::string, ros2_knowledge_graph_msgs::msg::Node> nodes_map;
    for(auto robot : robots) {
      auto want_to_see_edges = graph_->get_edges_from_node_by_data(robot.node_name, "want_to_see");
      for(auto edge : want_to_see_edges) {
        nodes_map[edge.target_node_id] = graph_->get_node(edge.target_node_id).value();
      }
    }

    /* Extract target_nodes from nodes_map. */
    std::vector<ros2_knowledge_graph_msgs::msg::Node> all_targets_nodes;
    for(auto pair : nodes_map) {
      all_targets_nodes.push_back(pair.second);
    }

    /* Recruiting the robots that are idle after liberating the busy ones that have finished. */
    nodes_vector free_robots;
    for (auto robot : robots) {
      auto target_edge = graph_->get_edges_from_node_by_data(robot.node_name, "target");

      if (target_edge.size() == 0) {
        /* No target, so it is idling. */

        free_robots.push_back(robot);

      } else if(nodes_map.count(target_edge[0].target_node_id) == 1) {
        /* It has a valid target(with the want_to_see edge). */

        auto watching_edge = graph_->get_edges_from_node_by_data(robot.node_name, "watching");
        if ((watching_edge.size() == 1) && (watching_edge[0].target_node_id == target_edge[0].target_node_id)) {
          /* The robot is watching the target. */

          auto count_started = get_property<bool>(robot, "count_started");
          if (!count_started.has_value()) {
            add_property(robot, "count_started", false);
            graph_->update_node(robot);
            count_started = get_property<bool>(robot, "count_started");
          }
          
          if (count_started.value()) {

            if (robots.size() < all_targets_nodes.size()) {
              /* Liberating the robot if the time has passed, only if there are more objects than robots. */

              double actual_time = shared_from_this()->now().seconds();
              auto started_time = get_property<double>(robot, "ros_time_started_watching");

              if ((actual_time - started_time.value()) > SECONDS_TO_STARE) {
                liberate_robot(robot, target_edge, free_robots);
              }
            }
            

          } else {
            /* First time watching it, saving current ros time in the corresponding property. */
            add_property(robot, "ros_time_started_watching", shared_from_this()->now().seconds());
            add_property(robot, "count_started", true);
            graph_->update_node(robot);
          } 
        }
      } else {
        /* No want_to_see edge. */
        liberate_robot(robot, target_edge, free_robots);
      }
    }


    /*
     * Map with the last time the object was observed and the corresponding node.
     * Since maps are automatically sorted, the oldest one is always going to be in the first position.
     */
    std::map<double, ros2_knowledge_graph_msgs::msg::Node> available_nodes;
    for(auto target_node : all_targets_nodes) {
      auto is_available = get_property<bool>(target_node, "available");
      if (!is_available.has_value()) {
        add_property(target_node, "available", true);
        graph_->update_node(target_node);
        is_available = get_property<bool>(target_node, "available");
      }
      if (is_available.value()) {
        auto ros_time = get_property<double>(target_node, "ros_time");
        if (!ros_time.has_value()) {
          add_property(target_node, "ros_time", shared_from_this()->now().seconds());
          graph_->update_node(target_node);
          ros_time = get_property<double>(target_node, "ros_time");
        }
        available_nodes[ros_time.value()] = target_node;
      }
    }

    //RCLCPP_INFO(get_logger(), "available_nodes:");
    for (auto pair : available_nodes) {
      //RCLCPP_INFO(get_logger(), "%s", ros2_knowledge_graph::to_string(pair.second).c_str());
    }


    /* Try to get a target for each free robot. */
    for (size_t i = 0; (i < free_robots.size()) && (available_nodes.size() > 0); i++) {
      auto target_node = available_nodes.begin()->second;
      available_nodes.erase(available_nodes.begin());
      add_property(target_node, "available", false);
      graph_->update_node(target_node);
      graph_->update_edge(new_edge<std::string>(free_robots[i].node_name, target_node.node_name, "target"));

      RCLCPP_INFO(get_logger(), "%s ------------> %s", free_robots[i].node_name.c_str(), target_node.node_name.c_str());
    }
    


    
    return;
  }

  void FocusSelector::fake_watching_updater() {
    /* Getting all robot nodes. */
    const nodes_vector robots = get_nodes_by_class("Robot");

    for (auto robot : robots) {
      auto target_edge = graph_->get_edges_from_node_by_data(robot.node_name, "target");
      auto watching_edge = graph_->get_edges_from_node_by_data(robot.node_name, "watching");

      /* If it has a target. */
      if (target_edge.size() == 1) {

        if ((watching_edge.size() == 1) && (watching_edge[0].target_node_id != target_edge[0].target_node_id)) {
          RCLCPP_INFO(get_logger(), "*removing watching edge to another object*");
          graph_->remove_edge(watching_edge[0]);
        }
        

        /* If it is not watching the target. */
        if (((watching_edge.size() == 1) && (watching_edge[0].target_node_id != target_edge[0].target_node_id))
              || (watching_edge.size() == 0)) {
          
          if (init_times_.count(robot.node_name) == 1) {
            double actual_time = shared_from_this()->now().seconds();
            double started_time = init_times_[robot.node_name];
            if ((actual_time - started_time) > SECONDS_TAKEN_TO_WATCH) {
              RCLCPP_INFO(get_logger(), "*time has passed, adding watching edge*");
              graph_->update_edge(new_edge<std::string>(robot.node_name, target_edge[0].target_node_id , "watching"));
              init_times_.erase(robot.node_name);
            }
            
          } else {
            RCLCPP_INFO(get_logger(), "*Starting timer*");
            init_times_[robot.node_name] = shared_from_this()->now().seconds();
          }
        }
      
      } else if (watching_edge.size() == 1) {
        RCLCPP_INFO(get_logger(), "*removing residual watching edge*");
        graph_->remove_edge(watching_edge[0]);
      }
    }
  }

  const std::vector<ros2_knowledge_graph_msgs::msg::Node>
  FocusSelector::get_nodes_by_class(const std::string & class_name)
  {
    const std::vector<std::string> node_names = graph_->get_node_names();
    std::vector<ros2_knowledge_graph_msgs::msg::Node> nodes;
    for (auto node_name : node_names) {
      nodes.push_back(graph_->get_node(node_name).value());
    }

    std::vector<ros2_knowledge_graph_msgs::msg::Node> result;
    for (const auto & node : nodes) {
      if (class_name == node.node_class) {
        result.push_back(node);
      }
    }
    return result;
  }

  bool FocusSelector::exist_edge(std::string source, std::string target, std::string content) {
    auto edges = graph_->get_edges<std::string>(source, target);
    
    for (auto edge : edges) {
      if ((edge.content.type == 4) && (edge.content.string_value == content)) {
        return true;
      }
    }

    return false;
  }

}   // namespace focus_selector