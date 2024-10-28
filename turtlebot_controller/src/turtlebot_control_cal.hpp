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
#include "turtlebot_sensorprocessing.hpp"


class turtlebot_control
{
public:
    /**
    @brief Constructor for the Control class.
    */
    turtlebot_control();

    void updateControlParam(geometry_msgs::msg::Point temp_lookahead, double temp_distanceToDestination, nav_msgs::msg::Odometry temp_odom, sensor_msgs::msg::LaserScan temp_lidar);

    geometry_msgs::msg::Twist generate_trajectory();
    
    double collisionDetection();

    bool goal_hit(geometry_msgs::msg::Point temp_goal, nav_msgs::msg::Odometry temp_odom);

    double distanceToGoal();

    double angleToGoal(nav_msgs::msg::Odometry temp_odom, geometry_msgs::msg::Point temp_goal);

    double steeringPID(geometry_msgs::msg::Point temp_goal, nav_msgs::msg::Odometry temp_odom);

    double velocityPID();



 

private:
    geometry_msgs::msg::Point goal;
    nav_msgs::msg::Odometry odom;
    sensor_msgs::msg::LaserScan lidar;
    double distanceToDestination;

    // Calculation variable declaration
    double Kp_;
    double Ki_;
    double Kd_;
    double toleranceDistance;
    double integral_;
    double prev_error_;

    double Kp_h;
    double Ki_h;
    double Kd_h;
    double toleranceAngle;
    double heading_integral_;
    double prev_heading_error_;

    double maxVelx;
    double maxVelz;

    double maxIntegral;
    double maxHeadingIntegral;
    double integralResetCount;

    std::vector<double> xPlot;
    std::vector<double> zPlot;
    std::vector<double> velPlot;
    nav_msgs::msg::Odometry prevOdom;

    Turtlebot_SensorProcessing ObjectDetection;

};

#endif // CONTROL_HPP