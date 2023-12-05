#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tutorial_interfaces/srv/get_waypoints.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcutils/cmdline_parser.h"

using namespace std::chrono_literals;

class WaypointsClientNode : public rclcpp::Node {
    public:
        WaypointsClientNode() : Node("waypoints_client") {
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&WaypointsClientNode::odom_callback, this, std::placeholders::_1));

            timer_ = create_wall_timer(10s, std::bind(&WaypointsClientNode::timer_callback, this));
            waypoints_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10);

            waypoints_client_ = this->create_client<tutorial_interfaces::srv::GetWaypoints>("waypoints_service");

            goal_pose_.x = -4.5; // Set goal pose {10, 10, 0}
            goal_pose_.y = 3.0;
            goal_pose_.z = 0.0;
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            // Create a request with the received start pose and goal pose
            start_pose_.x = msg->pose.pose.position.x;
            start_pose_.y = msg->pose.pose.position.y;
            start_pose_.z = msg->pose.pose.position.z;
            // RCLCPP_INFO(this->get_logger(), "Odom recieved");
            if (waypoints_) {
                if (waypoints_->valid && way_ind < waypoints_->waypoints.size()) {
                    const auto waypoint = waypoints_->waypoints[way_ind];
                    if (way_ind == 0){ 
                        waypoints_publisher_->publish(waypoint);
                        way_ind++;
                    } else {
                        double distance = sqrt(pow(waypoints_->waypoints[way_ind - 1].pose.position.x - start_pose_.x, 2) + pow(waypoints_->waypoints[way_ind-1].pose.position.y - start_pose_.y, 2));
                        if (distance < 0.5) {
                            RCLCPP_INFO(this->get_logger(), "Publishing waypoint to /waypoints");
                            waypoints_publisher_->publish(waypoint);
                            way_ind++;
                        }
                    }

                }
            }
        }

        void timer_callback() {
            // Create a request with the received start pose and goal pose
            if (!waypoints_) {
                auto request = std::make_shared<tutorial_interfaces::srv::GetWaypoints::Request>();
                geometry_msgs::msg::Point start;
                start.x = start_pose_.x;
                start.y = start_pose_.y;
                start.z = start_pose_.z;
                request->start = start;
                request->goal = goal_pose_;

                // Send request to waypoints_service
                while (!waypoints_client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) return;
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }

                using ServiceResponseFuture = rclcpp::Client<tutorial_interfaces::srv::GetWaypoints>::SharedFuture;
                auto response_received_callback = [this](ServiceResponseFuture future) {
                    auto result = future.get();
                    if (result->valid) waypoints_ = result;
                };
                auto future_result = waypoints_client_->async_send_request(request, response_received_callback);
            }
        }

        rclcpp::TimerBase::SharedPtr timer_ = nullptr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoints_publisher_;
        rclcpp::Client<tutorial_interfaces::srv::GetWaypoints>::SharedPtr waypoints_client_;
        std::shared_ptr<tutorial_interfaces::srv::GetWaypoints::Response> waypoints_;
        int way_ind = 0;
        geometry_msgs::msg::Point goal_pose_;
        geometry_msgs::msg::Point start_pose_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
