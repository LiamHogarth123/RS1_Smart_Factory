#ifndef DEFAULT_TURTLEBOT_HPP
#define DEFAULT_TURTLEBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "marker_msgs/msg/marker.hpp"
// #include "std_msgs/msg/int16.hpp"
#include <string>
#include <mutex>

class DefaultTurtleBot : public rclcpp::Node
{
public:
    DefaultTurtleBot(const std::string& name);
    void SendCmdTb1(const geometry_msgs::msg::Twist instructions);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void LidaCallback(const sensor_msgs::msg::LaserScan::SharedPtr Msg);
    void RGBCallback(const sensor_msgs::msg::Image::SharedPtr Msg);
    void ImageDepthCallback(const sensor_msgs::msg::Image::SharedPtr Msg);
    void guiderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    // void tagCallback(const marker_msgs::msg::Marker::SharedPtr Msg);
    // void boundaryCallback(const std_msgs::msg::Int16::SharedPtr Msg);

    nav_msgs::msg::Odometry GetCurrentOdom();
    sensor_msgs::msg::Image GetCurrentUpdatedRGB();
    sensor_msgs::msg::LaserScan GetUpdatedLida();
    sensor_msgs::msg::Image GetUpdatedImageDepth();
    double GetCurrentSpeed();
    marker_msgs::msg::Marker GetARTag();
    // std_msgs::msg::Int16 GetBoundaryStatus();

private:
    std::string namespace_;
    std::shared_ptr<rclcpp::Node> node_; 


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub3_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub4_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub5_;
    rclcpp::Subscription<marker_msgs::msg::Marker>::SharedPtr sub6_;
    // rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub7_;

    std::mutex odom_locker_;
    std::mutex odom_locker2_;
    std::mutex RGB_locker_;
    std::mutex Lida_locker_;
    std::mutex ImageDepth_locker_;
    std::mutex marker_locker_;
    std::mutex boundary_locker_;

    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Odometry guider_odom_;
    sensor_msgs::msg::Image updated_rgb_;
    sensor_msgs::msg::LaserScan updated_lida_;
    sensor_msgs::msg::Image updated_image_depth_;
    marker_msgs::msg::Marker ar_tag_;
    // std_msgs::msg::Int16 boundary_status_;

    double current_speed_;
};

#endif // DEFAULT_TURTLEBOT_HPP