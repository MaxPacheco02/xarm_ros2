#include "xarm_planner/xarm_planner.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

geometry_msgs::msg::Pose target_pose;
const std::string x_name = "xarm6";
rclcpp::Node::SharedPtr node = nullptr;
xarm_planner::XArmPlanner planner;

void update(const geometry_msgs::msg::Pose & msg){
    planner.planPoseTarget(msg);
    planner.executePath();
    RCLCPP_INFO(node->get_logger(), "Goal: %f, %f, %f", 
    msg.position.x, msg.position.y, msg.position.z);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("xarm_follow_pose", node_options);
    planner = xarm_planner::XArmPlanner(node, x_name);

    RCLCPP_INFO(node->get_logger(), "xarm_follow_pose node start");

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub;
    target_pose_sub = node->create_subscription<geometry_msgs::msg::Pose>(
        "/xarm_target", 10, update);

    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "xarm_follow_pose over");
    return 0;
}