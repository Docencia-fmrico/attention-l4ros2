
// Copyright 2022 L4ROS2

#ifndef PERCEPTION_HPP_
#define PERCEPTION_HPP_

#include <memory>
#include <string>
#include <map>

#include "gazebo_msgs/srv/GetEntityState.hpp"
#include "gazebo_msgs/msg/LinkStates.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#define ATTENTION_RADIUS 5.0

namespace perception {

class Perception : public rclcpp_lifecycle::LifecycleNode
{
public:
    Perception();
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State & state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_configure(const rclcpp_lifecycle::State & state);
    void links_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state);

    void do_work();

private:
    double speed_;

    inline double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
    {
        return sqrt(
        (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
        (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
    };


    std::map<std::string, geometry_msgs::msg::Pose> locations_;
    geometry_msgs::msg::Pose tiago_base_link_pose_;

    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr preception_client_;
    

    //rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
    //geometry_msgs::msg::Pose current_pos_;

};

}   // namespace perception

#endif  // PERCEPTION_HPP_