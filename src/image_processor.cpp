#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "your_package/srv/image_metadata.hpp"
#include "opencv2/opencv.hpp"

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor") {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "your_image_topic", 10,
            std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1)
        );

        metadata_service_ = this->create_service<your_package::srv::ImageMetadata>(
            "image_metadata_service",
            std::bind(&ImageProcessor::metadata_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Process the received image using OpenCV
        cv::Mat cv_image = process_image(msg);

        // Store metadata
        auto image_metadata = extract_metadata(msg);
        metadata_[msg->header.stamp] = image_metadata;
    }

    cv::Mat process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Use OpenCV to process the image data in C++
        // Implement your image processing logic here
        // For example:
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        // ... Process the image using OpenCV
        return cv_image;
    }

    std::map<rclcpp::Time, your_package::srv::ImageMetadata::Response> metadata_;

    void metadata_service_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<your_package::srv::ImageMetadata::Request> request,
        const std::shared_ptr<your_package::srv::ImageMetadata::Response> response
    ) {
        // Handle metadata service requests
        auto requested_data = metadata_[request->image_id]; // Retrieve metadata based on image ID
        if (requested_data) {
            response->metadata = requested_data;
        } else {
            response->metadata = {};  // Return empty metadata if image ID not found
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Service<your_package::srv::ImageMetadata>::SharedPtr metadata_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
