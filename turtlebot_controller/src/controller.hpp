#ifndef TURTLEBOT_CONTROLLER_HPP
#define TURTLEBOT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "marker_msgs/msg/marker.hpp"

// #include "turtlebot.hpp"
#include "turtlebot_sensorprocessing.hpp"
#include "turtlebot_control_cal.hpp"

#include <std_msgs/msg/bool.hpp>


class Controller : public rclcpp::Node {
public:
    Controller(const std::string &namespace_param);
    void Default_state();
 

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();
    
    
    


    std::string namespace_param_;

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path::SharedPtr path_;
    size_t current_waypoint_;
    geometry_msgs::msg::Pose current_pose_;


    Turtlebot_SensorProcessing machine_vision_;
    turtlebot_control Turtlebot_GPS_;
    // std::unique_ptr<DefaultTurtleBot> 
    // DefaultTurtleBot* turtlebot_RosCom_; // Unique ownership   
    //flags
    bool NewPath_;
    bool shutdown_request_;


    //turtlebot_ros2 Communication
    void SendCmdTb1(const geometry_msgs::msg::Twist instructions);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void LidaCallback(const sensor_msgs::msg::LaserScan::SharedPtr Msg);
    void RGBCallback(const sensor_msgs::msg::Image::SharedPtr Msg);
    void ImageDepthCallback(const sensor_msgs::msg::Image::SharedPtr Msg);
    void guiderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void shut_downCallback(const std_msgs::msg::Bool::SharedPtr msg);


    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shut_down_request;



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

    #endif // TURTLEBOT_CONTROLLER_HPP
