#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <numeric>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>


class ur3_control
{
public:
    ur3_control();
};

#endif // CONTROL_HPP