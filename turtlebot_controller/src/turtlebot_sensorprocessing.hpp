#ifndef LASER_PROCESSING_HPP
#define LASER_PROCESSING_HPP

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <iostream>
#include <cmath>
#include <vector>


class Turtlebot_SensorProcessing
{
public:
    Turtlebot_SensorProcessing(); // Constructor declaration

    void NewData(const sensor_msgs::msg::LaserScan& temp_data);
    geometry_msgs::msg::Point polarToCart(unsigned int index);
    double findObstacle();
    std::vector<std::pair<float, int>> scanningRange(float scanRange);
    void PrintLaserSpec();

private:
    sensor_msgs::msg::LaserScan laserScan;
    double Turtlebot_min;
    double Turtlebot_max;
};

#endif // LASER_PROCESSING_HPP
