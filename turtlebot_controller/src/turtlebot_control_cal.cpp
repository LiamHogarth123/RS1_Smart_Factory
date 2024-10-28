#include "turtlebot_control_cal.hpp"


/**
 @file control.cpp
 @brief This file contains the implementation of the Control class.
*/

turtlebot_control::turtlebot_control()
{
    toleranceDistance = 0.1;
    toleranceAngle = 0.05;
    prev_error_ = 0;
    prev_heading_error_ = 0;

    Kp_ = 0.8;
    Ki_ = 0.1;
    Kd_ = 4.5;

    Kp_h = 0.5;
    Ki_h = 0.1;
    Kd_h = 2;

    maxVelx = 0.26; // m/s
    maxVelz = 1.82; // rad/s

    maxIntegral = 10;
    maxHeadingIntegral = 15;

    integral_ = 0;
    heading_integral_ = 0;
    integralResetCount = 15;

    prevOdom.pose.pose.position.x = 0;
    prevOdom.pose.pose.position.y = 0;
}

void turtlebot_control::updateControlParam(geometry_msgs::msg::Point temp_lookahead, double temp_distanceToDestination, nav_msgs::msg::Odometry temp_odom, sensor_msgs::msg::LaserScan temp_lidar)
{
    goal = temp_lookahead;
    odom = temp_odom;
    lidar = temp_lidar;
    distanceToDestination = temp_distanceToDestination;
}

geometry_msgs::msg::Twist turtlebot_control::generate_trajectory()
{
    geometry_msgs::msg::Twist cmd_vel;

    double velocityX = velocityPID();
    double velocityZ = steeringPID(goal, odom);

    //// OBJECT AVOIDANCE ////
    double obstacleMidpoint = collisionDetection();
    double avoidanceFactor = -0.08; // the value determining the rate of avoidance (lower is faster rate of change)
    integralResetCount++; // for integral reset at initial object detection

    std::cout << "MIDPOINT: " << obstacleMidpoint << std::endl;

    if (obstacleMidpoint > 0) {
        // std::cout << obstacleMidpoint << std::endl;
        std::cout << "avoiding object on left" << std::endl;
        velocityZ = avoidanceFactor/pow(obstacleMidpoint+0.23,2); 
        if (velocityZ < -maxVelz) {
            velocityZ = -maxVelz;
        }
        if (integralResetCount > 15){ // if object first detected then slows down by resetting integral so it builds back up speed
            integral_ = 0;
        }
        integralResetCount = 0;
        // velocityX *= 0.5;
    } else if (obstacleMidpoint < 0) {
        // std::cout << obstacleMidpoint << std::endl;
        std::cout << "avoiding object on right" << std::endl;
        velocityZ = avoidanceFactor/-pow(obstacleMidpoint-0.23,2);
        if (velocityZ > maxVelz) {
            velocityZ = maxVelz;
        }
        if (integralResetCount > 15){
            integral_ = 0;
        }
        integralResetCount = 0;
        // velocityX *= 0.5;
    }

    cmd_vel.linear.x = velocityX;
    cmd_vel.angular.z = velocityZ;

    return cmd_vel;
}

double turtlebot_control::velocityPID()
{
    double current_distance = distanceToGoal();

    double error = -(toleranceDistance - current_distance);

    integral_ += error;

    if (integral_ > maxIntegral)
    {
        integral_ = maxIntegral;
    }
    else if (integral_ < -maxIntegral)
    {
        integral_ = -maxIntegral;
    }

    double derivative = error - prev_error_;

    double control_command = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    control_command *= 0.2; // scaling down

    prev_error_ = error;

    if (fabs(distanceToGoal()) < toleranceDistance)
    {
        integral_ = 0;
    }

    if (control_command > maxVelx)
    {
        control_command = maxVelx;
    }

    if (distanceToDestination < 0.5)
    {
        control_command *= 0.8;
    }

    return control_command;
}

double turtlebot_control::steeringPID(geometry_msgs::msg::Point temp_goal, nav_msgs::msg::Odometry temp_odom)
{
    double current_heading = angleToGoal(temp_odom, temp_goal);
    double heading_error = -(toleranceAngle - current_heading);

    heading_integral_ += heading_error;

    if (heading_integral_ > maxHeadingIntegral)
    {
        heading_integral_ = maxHeadingIntegral;
    }
    else if (heading_integral_ < -maxHeadingIntegral)
    {
        heading_integral_ = -maxHeadingIntegral;
    }

    double heading_derivative = heading_error - prev_heading_error_;

    double angular_command = Kp_h * heading_error + Ki_h * heading_integral_ + Kd_h * heading_derivative;
    angular_command *= 0.6; // scaling down

    prev_heading_error_ = heading_error;

    if (fabs(current_heading) < toleranceAngle)
    {
        angular_command = 0;
        heading_integral_ = 0;
    }

    if (heading_error > 0)
    {
        angular_command = fabs(angular_command);  // Turn right
        if (angular_command > maxVelz)
        {
            angular_command = maxVelz;
        }
    }
    else
    {
        angular_command = -fabs(angular_command); // Turn left
        if (angular_command < -maxVelz)
        {
            angular_command = -maxVelz;
        }
    }

    return angular_command;
}

double turtlebot_control::collisionDetection() {

    ObjectDetection.NewData(lidar);
    double obstacleMidpoint = ObjectDetection.findObstacle();

    // if obstacle == 0 then does nothing
    if (obstacleMidpoint != 0) {
        // integral_ = 0;
        heading_integral_ = 0;
    }


    return obstacleMidpoint;
}

bool turtlebot_control::goal_hit(geometry_msgs::msg::Point temp_goal, nav_msgs::msg::Odometry temp_odom)
{
    if (distanceToGoal() <= toleranceDistance)
    {
        integral_ = 0;
        heading_integral_ = 0;
        return true;
    }
    else
    {
        return false;
    }
}

double turtlebot_control::distanceToGoal()
{
    double delta_x = goal.x - odom.pose.pose.position.x;
    double delta_y = goal.y - odom.pose.pose.position.y;
    double distance = sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));

    return distance;
}

double turtlebot_control::angleToGoal(nav_msgs::msg::Odometry temp_odom, geometry_msgs::msg::Point temp_goal)
{
    tf2::Quaternion current_orientation;
    tf2::fromMsg(temp_odom.pose.pose.orientation, current_orientation);

    current_orientation.normalize();

    tf2::Vector3 heading_vector(1, 0, 0);

    heading_vector = tf2::quatRotate(current_orientation, heading_vector);

    tf2::Vector3 goal_vector(temp_goal.x - temp_odom.pose.pose.position.x, temp_goal.y - temp_odom.pose.pose.position.y, 0);

    heading_vector.normalize();
    goal_vector.normalize();

    double angle = atan2(heading_vector.cross(goal_vector).z(), heading_vector.dot(goal_vector));

    return angle;
}

