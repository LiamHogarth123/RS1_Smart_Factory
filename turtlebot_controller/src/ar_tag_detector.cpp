#include "ar_tag_detection.hpp"

// Constructor
ARDetection::ARDetection() {
    // Initialize AprilTag detector with default parameters
    apriltag_ros::AprilTagDetector::Config config;
    detector_.setConfig(config);
}

// Function to process image and detect AR tags
std::vector<int> ARDetection::detectARTags(const sensor_msgs::msg::Image::SharedPtr image_msg) {
    // Convert the ROS image message to OpenCV format
    cv::Mat frame = convertImage(image_msg);

    // Detect AprilTags in the frame
    std::vector<apriltag_ros::AprilTagDetection> detections = detector_.detect(frame);

    std::vector<int> detected_ids;

    // Iterate through the detections and extract the IDs
    for (const auto& detection : detections) {
        detected_ids.push_back(detection.id);  // Add the ID to the detected_ids vector
    }

    // Return the list of detected AR tags
    return detected_ids;
}

// Helper function to convert ROS image to OpenCV Mat format
cv::Mat ARDetection::convertImage(const sensor_msgs::msg::Image::SharedPtr image_msg) {
    try {
        return cv_bridge::toCvCopy(image_msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ARDetection"), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}
