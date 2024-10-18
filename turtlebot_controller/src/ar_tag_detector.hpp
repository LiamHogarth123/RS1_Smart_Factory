#ifndef AR_DETECTION_HPP
#define AR_DETECTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag_ros/apriltag_detector.hpp>

class ARDetection {
public:
    ARDetection();  // Constructor

    // Function to process the image and detect AR tags
    std::vector<int> detectARTags(const sensor_msgs::msg::Image::SharedPtr image_msg);

private:
    // AprilTag detector object
    apriltag_ros::AprilTagDetector detector_;

    // Helper function to convert ROS image to OpenCV format
    cv::Mat convertImage(const sensor_msgs::msg::Image::SharedPtr image_msg);
};

#endif // AR_DETECTION_HPP
