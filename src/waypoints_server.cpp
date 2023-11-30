#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tutorial_interfaces/srv/get_waypoints.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "processImage.h"
#include <memory>

class WaypointServerNode : public rclcpp::Node {
public:
    WaypointServerNode() : Node("waypoints_server") {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_uav/image_raw", 10,
            std::bind(&WaypointServerNode::image_callback, this, std::placeholders::_1)
        );

        waypoints_service_ = this->create_service<tutorial_interfaces::srv::GetWaypoints>(
            "waypoints_service",
            std::bind(&WaypointServerNode::metadata_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
        );
        maxNodes = 800;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (warming > 7) {
            if (GridSpacePtr->getPoints().size() < maxNodes) { 
                RCLCPP_INFO(this->get_logger(), "SERVER PROCESSING IMAGE");
                GridSpacePtr->setGrid(process_image(msg));
                GridSpacePtr->runPRM({0, 0}, {1, 1}, 100);
                // GridSpacePtr->showPRM(cv_ptr->image);
                // std::string filename =  "PRM" + std::to_string(warming) + ".png";
                // bool success = cv::imwrite(filename, cv_ptr->image);
            } 
            // else GridSpacePtr->showPRM(cv_ptr->image);
        }
        warming++;
    }

    cv::Mat process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::Scalar lowerColor = cv::Scalar(155, 155, 155);  // Lower bound for (155, 155, 155)
        cv::Scalar upperColor = cv::Scalar(155, 155, 155); 
        cv::Mat mask;
        cv::inRange(cv_image, lowerColor, upperColor, mask);
        // bool success = cv::imwrite("image.png", cv_image);
        // bool success1 = cv::imwrite("mask.png", mask);
        // cv::imshow("CV Image", mask);
        // cv::waitKey(0);
        return mask;
    }

    void metadata_service_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<tutorial_interfaces::srv::GetWaypoints::Request> request,
        const std::shared_ptr<tutorial_interfaces::srv::GetWaypoints::Response> response
    ) {
        // Handle metadata service requests
        geometry_msgs::msg::Point start = request->start;
        geometry_msgs::msg::Point goal = request->goal;
        Path path = GridSpacePtr->getWaypoints({start.x, start.y}, {goal.x, goal.y});
        response->valid = path.valid;
        RCLCPP_INFO(this->get_logger(), "SERVER WAYPOINTS RESPONSE");

    for (size_t i = 1; i < path.waypoints.size(); ++i) {
        // Extract previous and current waypoints
        const std::pair<double, double>& prev_waypoint = path.waypoints[i - 1];
        const std::pair<double, double>& curr_waypoint = path.waypoints[i];

        // Create PoseStamped message
        auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_msg->header.frame_id = "map"; // Adjust the frame ID as necessary

        // Set position
        pose_msg->pose.position.x = curr_waypoint.first;
        pose_msg->pose.position.y = curr_waypoint.second;
        pose_msg->pose.position.z = 0.0;

        // Calculate orientation based on direction vector
        double dx = curr_waypoint.first - prev_waypoint.first;
        double dy = curr_waypoint.second - prev_waypoint.second;
        double yaw = std::atan2(dy, dx);

        pose_msg->pose.orientation.w = std::cos(yaw / 2.0);
        pose_msg->pose.orientation.z = std::sin(yaw / 2.0);
        response->waypoints.push_back(*pose_msg);
    }

        // Retrieve metadata based on image ID
        // response->waypoints.push_back(start);
        // RCLCPP_INFO(
        //     rclcpp::get_logger("waypoints_server"),
        //     "Constructed waypoints: {%f, %f, %f}, {%f, %f, %f}, {%f, %f, %f}",
        //     response->waypoints[0].x, response->waypoints[0].y, response->waypoints[0].z,
        //     response->waypoints[1].x, response->waypoints[1].y, response->waypoints[1].z,
        // );
    }
    
    bool open = true;
    int warming = 0;
    cv_bridge::CvImagePtr cv_ptr;
    std::map<rclcpp::Time, int> metadata_;
    uint32_t maxNodes;
    std::shared_ptr<GridSpace> GridSpacePtr = std::make_shared<GridSpace>(maxNodes);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Service<tutorial_interfaces::srv::GetWaypoints>::SharedPtr waypoints_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
