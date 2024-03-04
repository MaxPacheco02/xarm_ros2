#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
    TrajectoryPlannerNode() : Node("trajectory_planner_node")
    {
        using namespace std::placeholders;

        target_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            "/xarm_target", 10, 
            [this](const geometry_msgs::msg::Pose &msg){
                this->target_pose = msg;
            });

        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/mk_arr", 10);

        timer_ =
            this->create_wall_timer(1000ms, std::bind(&TrajectoryPlannerNode::update, this));
    }


private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

    geometry_msgs::msg::Pose target_pose;
    visualization_msgs::msg::MarkerArray marker_arr;
    visualization_msgs::msg::Marker marker;

    int i = 0;

    void update() {
        // this->marker_arr.markers.clear();
        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(1).g(0).b(0).a(1); 
        marker.header.frame_id = "world";
        marker.id = (i++);
        marker.type = 2;
        marker.action = 0;
        marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.03).y(0.03).z(0.03); 
        marker.pose.position.x = this->target_pose.position.x;
        marker.pose.position.y = this->target_pose.position.y;
        marker.pose.position.z = this->target_pose.position.z;
        this->marker_arr.markers.push_back(marker);
        marker_pub->publish(this->marker_arr);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
