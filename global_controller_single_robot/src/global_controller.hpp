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
#include <std_msgs/msg/bool.hpp>



#include "prm2.hpp"
#include "task_allocation.hpp"

// #include "turtlebot.hpp"
// #include "turtlebot_sensorprocessing.hpp"
// #include "turtlebot_control_cal.hpp"

class Global_Controller : public rclcpp::Node {
public:
    Global_Controller(const std::string &namespace_param);
    void Default_state();
    void statusCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void publishTrajectory(std::vector<geometry_msgs::msg::Point> goals);
 

private:
  
    
    bool robot_status;

    nav_msgs::msg::OccupancyGrid map;


    std::string namespace_param_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Shut_down_request;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_status_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;


    Task_Allocation TA;
    Path_Planner GPS;
    bool map_data_recieved;







};

    #endif // TURTLEBOT_CONTROLLER_HPP
