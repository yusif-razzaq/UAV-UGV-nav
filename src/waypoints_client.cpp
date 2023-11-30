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

            // Send request to waypoints_service
            // while (!waypoints_client_->wait_for_service(std::chrono::seconds(1))) {
            //     if (!rclcpp::ok()) {
            //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service to appear.");
            //         return;
            //     }
            //     RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            // }

            // auto result_future = waypoints_client_->async_send_request(request);
            // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            //     auto result = result_future.get();
            //     if (result) {
            //         // Publish received waypoints response to "/waypoints" topic
            //         for (const auto& point : result->waypoints) {
            //             waypoints_publisher_->publish(point);
            //             RCLCPP_INFO(this->get_logger(), "Publishing waypoint");
            //         }
            //     } else {
            //         RCLCPP_ERROR(this->get_logger(), "Service call failed.");
            //     }
            // } else {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
            // }
        }

        void timer_callback() {
            // Create a request with the received start pose and goal pose
            if (open) {
                auto request = std::make_shared<tutorial_interfaces::srv::GetWaypoints::Request>();
                geometry_msgs::msg::Point start;
                start.x = start_pose_.x;
                start.y = start_pose_.y;
                start.z = start_pose_.z;
                request->start = start;
                request->goal = goal_pose_;

                // Send request to waypoints_service
                while (!waypoints_client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service to appear.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }

                using ServiceResponseFuture = rclcpp::Client<tutorial_interfaces::srv::GetWaypoints>::SharedFuture;
                auto response_received_callback = [this](ServiceResponseFuture future) {
                    auto result = future.get();
                    for (const auto& pose : result->waypoints) {
                        open = false;
                        waypoints_publisher_->publish(pose);
                        RCLCPP_INFO(this->get_logger(), "Publishing waypoint to /waypoints");
                        rclcpp::sleep_for(std::chrono::milliseconds(3750));
                    }
                };
                RCLCPP_INFO(this->get_logger(), "CLIENT SENDING REQUEST");

                auto future_result = waypoints_client_->async_send_request(request, response_received_callback);

                // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
                //     auto result = result_future.get();
                //     if (result) {
                //         // Publish received waypoints response to "/waypoints" topic
                //         for (const auto& point : result->waypoints) {
                //             waypoints_publisher_->publish(point);
                //             RCLCPP_INFO(this->get_logger(), "Publishing waypoint");
                //         }
                //     } else {
                //         RCLCPP_ERROR(this->get_logger(), "Service call failed.");
                //     }
                // } else {
                //     RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
                // }
            }
        }

        bool open = true;
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoints_publisher_;
        rclcpp::Client<tutorial_interfaces::srv::GetWaypoints>::SharedPtr waypoints_client_;
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
