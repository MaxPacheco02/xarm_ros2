#include "xarm_planner/xarm_planner.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

nav_msgs::msg::Path path;
const std::string x_name = "xarm6";
rclcpp::Node::SharedPtr node = nullptr;
xarm_planner::XArmPlanner planner;

void update(const nav_msgs::msg::Path & msg){
    for(int i = 0 ; i < msg.poses.size() ; i++){
        RCLCPP_INFO(node->get_logger(), "Goal: %f, %f, %f", 
        msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.position.z);
        planner.planPoseTarget(msg.poses[i].pose);
        planner.executePath();
        rclcpp::sleep_for(150ms);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("xarm_follow_pose", node_options);
    planner = xarm_planner::XArmPlanner(node, x_name);

    RCLCPP_INFO(node->get_logger(), "xarm_follow_pose node start");

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_pose_sub;
    target_pose_sub = node->create_subscription<nav_msgs::msg::Path>(
        "/xarm_planned_path", 10, update);

    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "xarm_follow_pose over");
    return 0;
}