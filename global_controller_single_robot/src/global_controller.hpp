#ifndef TURTLEBOT_CONTROLLER_HPP
#define TURTLEBOT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include "marker_msgs/msg/marker.hpp"
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int32.hpp"


#include "turtlebot_manager.hpp"
#include "prm2.hpp"
#include "task_allocation.hpp"

// #include "turtlebot.hpp"
// #include "turtlebot_sensorprocessing.hpp"
// #include "turtlebot_control_cal.hpp"

class Global_Controller : public rclcpp::Node {
public:
    Global_Controller(const int &num_robots);
    void Default_state();
    void Default_state_multi();
    void Default_state_multi_complex();
    void statusCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void publishTrajectory(std::vector<geometry_msgs::msg::Point> goals);
    void publish_package_status(int package_id, int status_code);

 

private:
  
    bool Process_Package_AR_info(int Found_tag, int expected_tag);
    bool robot_status;

    nav_msgs::msg::OccupancyGrid map;


    std::string namespace_param_;

    //Ros pub and subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_status_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscription_;


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Shut_down_request;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    


    Task_Allocation TA;
    Path_Planner GPS;
    bool map_data_recieved;







};

    #endif // TURTLEBOT_CONTROLLER_HPP
