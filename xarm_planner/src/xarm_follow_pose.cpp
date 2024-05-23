#include "xarm_planner/xarm_planner.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

void updateObj(const geometry_msgs::msg::PoseStamped & msg){

    RCLCPP_INFO(node->get_logger(), "Object: %lf, %lf, %lf", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    planner.planPoseTarget(msg.pose);
    planner.executePath();
    rclcpp::sleep_for(150ms);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("xarm_follow_pose", node_options);
    planner = xarm_planner::XArmPlanner(node, x_name);

    //rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_pose_sub;
    //target_pose_sub = node->create_subscription<nav_msgs::msg::Path>(
    //    "/xarm_planned_path", 10, update);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obj_pos_sub;
    obj_pos_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/xarm_obj_pose", 10, updateObj);

    rclcpp::spin(node);
    return 0;
}
