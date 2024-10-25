#include "turtlebot_sensorprocessing.hpp"


Turtlebot_SensorProcessing::Turtlebot_SensorProcessing()
{
    Turtlebot_min = 0.02;
    Turtlebot_max = 0.08;
}

void Turtlebot_SensorProcessing::NewData(const sensor_msgs::msg::LaserScan& temp_data)
{
    laserScan = temp_data;
}

void Turtlebot_SensorProcessing::PrintLaserSpec()
{
    std::cout << "min" << std::endl;
    std::cout << laserScan.angle_min << std::endl;
    std::cout << "max" << std::endl;
    std::cout << laserScan.angle_max << std::endl;
    std::cout << "increment" << std::endl;
    std::cout << laserScan.angle_increment << std::endl;
}

geometry_msgs::msg::Point Turtlebot_SensorProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan.angle_min + laserScan.angle_increment * index;
    float range = laserScan.ranges.at(index);
    geometry_msgs::msg::Point cart;
    cart.x = static_cast<double>(range * cos(angle));
    cart.y = static_cast<double>(range * sin(angle));
    return cart;
}

double Turtlebot_SensorProcessing::findObstacle() {
    double midpoint = 0;

    return midpoint;
}

std::vector<std::pair<float, int>> Turtlebot_SensorProcessing::scanningRange(float scanRange)
{
    std::vector<std::pair<float, int>> combinedVector ;
    return combinedVector;
}




// ADD AR TAG REC