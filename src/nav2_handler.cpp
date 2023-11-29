#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class Nav2Handler : public rclcpp::Node {
public:
    Nav2Handler() : Node("nav2_handler") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/waypoints",
            10,
            std::bind(&Nav2Handler::waypoint_callback, this, std::placeholders::_1)
        );

        nav_goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10
        );
    }

private:
    void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received waypoint: %f, %f, %f", msg->x, msg->y, msg->z);

        auto nav_goal = std::make_unique<geometry_msgs::msg::PoseStamped>();
        nav_goal->header.frame_id = "map"; // Adjust the frame ID if needed
        nav_goal->pose.position.x = msg->x;
        nav_goal->pose.position.y = msg->y;
        nav_goal->pose.position.z = msg->z;

        // Assuming no orientation information is provided, set a default orientation
        nav_goal->pose.orientation.w = 0.0; // For instance, setting a default orientation

        // Publish the navigation goal
        nav_goal_publisher_->publish(std::move(nav_goal));
        RCLCPP_INFO(this->get_logger(), "Sent navigation goal to Nav2");
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto nav2_handler = std::make_shared<Nav2Handler>();
    rclcpp::spin(nav2_handler);
    rclcpp::shutdown();
    return 0;
}
