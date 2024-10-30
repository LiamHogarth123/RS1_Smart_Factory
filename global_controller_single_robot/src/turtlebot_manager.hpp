#ifndef DEFAULT_TURTLEBOT_HPP
#define DEFAULT_TURTLEBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include "marker_msgs/msg/marker.hpp"
// #include "std_msgs/msg/int16.hpp"
#include "nav_msgs/msg/path.hpp"
#include "warehouse_robot_msgs/msg/robot_data.hpp"

#include <string>
#include <mutex>

class TurtleBotManager : public rclcpp::Node
{
public:
    TurtleBotManager(const std::string& name, int id);
    void robot_info_Callback(const warehouse_robot_msgs::msg::RobotData::SharedPtr msg);
    // void robot_info_Callback(const warehouse_robot_msgs::msg::RobotData::SharedPtr msg);
    void robot_info_Callback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void customOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    void publishTrajectory(std::vector<geometry_msgs::msg::Point> goals);


    nav_msgs::msg::Odometry GetCurrentOdom();
    double GetCurrentSpeed();
    int GetARTag();
    std::string getStatus();
    bool get_status_bool();
    

    bool odom_recieved;


private:
    std::string namespace_;
    std::shared_ptr<rclcpp::Node> node_; 
    bool data_recieved;


    rclcpp::Subscription<warehouse_robot_msgs::msg::RobotData>::SharedPtr robot_info_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr back_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;



    std::mutex odom_locker_;
    
    

    nav_msgs::msg::Odometry current_odom_;
    int status;
    int AR_tag;

    double current_speed_;

};

#endif // DEFAULT_TURTLEBOT_HPP