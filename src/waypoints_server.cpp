#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tutorial_interfaces/srv/get_waypoints.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

class WaypointServerNode : public rclcpp::Node {
public:
    WaypointServerNode() : Node("waypoints_server") {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10,
            std::bind(&WaypointServerNode::image_callback, this, std::placeholders::_1)
        );

        waypoints_service_ = this->create_service<tutorial_interfaces::srv::GetWaypoints>(
            "waypoints_service",
            std::bind(&WaypointServerNode::metadata_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
        );
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Process the received image using OpenCV
        cv::Mat cv_image = process_image(msg);

        // Store metadata
        // auto image_metadata = extract_metadata(msg);
        metadata_[msg->header.stamp] = 1;
    }

    cv::Mat process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Use OpenCV to process the image data in C++
        // Implement your image processing logic here
        // For example:
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // ... Process the image using OpenCV
        return cv_image;
    }

    void metadata_service_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<tutorial_interfaces::srv::GetWaypoints::Request> request,
        const std::shared_ptr<tutorial_interfaces::srv::GetWaypoints::Response> response
    ) {
        // Handle metadata service requests
        geometry_msgs::msg::Point start = request->start;
        geometry_msgs::msg::Point goal = request->goal;
        // Retrieve metadata based on image ID
        // response->waypoints.push_back(start);
        response->waypoints.push_back(goal);
        // RCLCPP_INFO(
        //     rclcpp::get_logger("waypoints_server"),
        //     "Constructed waypoints: {%f, %f, %f}, {%f, %f, %f}, {%f, %f, %f}",
        //     response->waypoints[0].x, response->waypoints[0].y, response->waypoints[0].z,
        //     response->waypoints[1].x, response->waypoints[1].y, response->waypoints[1].z,
        // );
    }
    
    cv_bridge::CvImagePtr cv_ptr;
    std::map<rclcpp::Time, int> metadata_;
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
