

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "marker_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"  
#include <nav_msgs/msg/odometry.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


// #include "turtlebot.hpp"
// #include "turtlebot_sensorprocessing.hpp"
// #include "turtlebot_control_cal.hpp"



struct turtlebot_job {
    int id;
    geometry_msgs::msg::Point package_location;
    geometry_msgs::msg::Point delivery_location;
};

class Task_Allocation {
public:
    Task_Allocation();
    std::vector<geometry_msgs::msg::Point> get_job_list();
    void Load_job_list_txt();

    std::vector<turtlebot_job> get_Job_List(std::vector<nav_msgs::msg::Odometry> turtlebot_starts);

    std::vector<std::vector<turtlebot_job>> optimise_turtlebot_jobs(int num_robot);

    std::vector<turtlebot_job> PackageSort(const geometry_msgs::msg::Point& current_location);



    double calculateEuclideanDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);


private:
    std::string namespace_param_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::vector<turtlebot_job> turtlebot_job_list;

    std::string job_list_file_path;








};

 