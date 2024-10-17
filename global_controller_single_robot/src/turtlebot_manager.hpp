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
    void robot_info_Callback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void publishTrajectory(std::vector<geometry_msgs::msg::Point> goals);

    

    nav_msgs::msg::Odometry GetCurrentOdom();
    double GetCurrentSpeed();
    marker_msgs::msg::Marker GetARTag();
    
    // std_msgs::msg::Int16 GetBoundaryStatus();

private:
    std::string namespace_;
    std::shared_ptr<rclcpp::Node> node_; 


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;


    std::mutex odom_locker_;
    
    nav_msgs::msg::Odometry current_odom_;
    marker_msgs::msg::Marker ar_tag_;
    // std_msgs::msg::Int16 boundary_status_;



    int status;
    int AR_tag;

    double current_speed_;
};

#endif // DEFAULT_TURTLEBOT_HPP