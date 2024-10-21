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
    std::vector<std::pair<float, int>> scannedRange = scanningRange(90);
    double distance = 0;
    double midpoint = 0;
    int objectCount = 0;
    geometry_msgs::msg::Point obstacleStart;

    for (int i = 0; i < scannedRange.size(); i++)
    {
        if (scannedRange[i].first < 0.6)
        {
            objectCount++;
            int ObjStartingPt = i;

            while (i < scannedRange.size() && scannedRange[i].first < 0.6)
            {
                i++;
            }

            if (objectCount == 1)
            {
                obstacleStart = polarToCart(scannedRange[ObjStartingPt].second);
            }

            if (objectCount > 0)
            {
                geometry_msgs::msg::Point obstacleEnd = polarToCart(scannedRange[i - 1].second);

                distance = sqrt(pow((obstacleStart.x - obstacleEnd.x), 2) + pow((obstacleStart.y - obstacleEnd.y), 2));

                midpoint = (obstacleStart.y + obstacleEnd.y) / 2.0;
            }
        }
    }

    return midpoint;
}

std::vector<std::pair<float, int>> Turtlebot_SensorProcessing::scanningRange(float scanRange)
{
    float scanSize = laserScan.ranges.size();
    float degreeIndex = scanSize / 360;
    float scanIndex = round((scanRange / 2) * degreeIndex);

    std::vector<std::pair<float, int>> scanPosDirection;
    std::vector<std::pair<float, int>> scanNegDirection;

    for (int i = 0; i < scanIndex; ++i)
    {
        scanPosDirection.push_back(std::make_pair(laserScan.ranges[i], i));
    }

    for (int i = laserScan.ranges.size() - scanIndex; i < laserScan.ranges.size(); ++i)
    {
        scanNegDirection.push_back(std::make_pair(laserScan.ranges[i], i));
    }

    std::vector<std::pair<float, int>> combinedVector = scanNegDirection;
    for (const auto& element : scanPosDirection)
    {
        combinedVector.push_back(element);
    }

    return combinedVector;
}




// ADD AR TAG REC