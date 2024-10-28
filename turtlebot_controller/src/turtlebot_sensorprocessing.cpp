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

double Turtlebot_SensorProcessing::findObstacle(){

    std::vector<std::pair<float, int>> scannedRange = scanningRange(90);
    double distance = 0;
    double midpoint = 0;
    int objectCount = 0; // counts how many objects in the scan
    geometry_msgs::msg::Point obstacleStart;


    for (int i = 0; i < scannedRange.size(); i++) {

        if (scannedRange[i].first < 0.6) {
            objectCount++;
            int ObjStartingPt = i;
        
            while (i < scannedRange.size() && scannedRange[i].first < 0.6) {

                i++; // move to the end of the object
            }

            if (objectCount == 1) {
                obstacleStart = polarToCart(scannedRange[ObjStartingPt].second);
            }

            if (objectCount > 0) {
                geometry_msgs::msg::Point obstacleEnd = polarToCart(scannedRange[i - 1].second);

                distance = sqrt(pow((obstacleStart.x - obstacleEnd.x), 2) + pow((obstacleStart.y - obstacleEnd.y), 2));

                midpoint = (obstacleStart.y + obstacleEnd.y) / 2.0;
            }
        }

    }

    // std::cout << "---------------------------------------------" << std::endl;
    // std::cout << "distance: " << distance << std::endl;
    // std::cout << "midpoint: " << midpoint << std::endl;
    // std::cout << "polarToCart: " << polarToCart(1) << std::endl;
    // std::cout << "---------------------------------------------" << std::endl;
    
    return midpoint;

}

std::vector<std::pair<float, int>> Turtlebot_SensorProcessing::scanningRange(float scanRange){ //scans from right to left

    float scanSize = laserScan.ranges.size();
    float degreeIndex = scanSize / 360;
    float scanIndex = round((scanRange/2) * degreeIndex); // The index of the scan at scanRange (degrees), scanRange/2 as there is left and right of 0 degrees
    

    // std::cout << "LaserScan.ranges: ";
    //     for (const auto& element : laserScan.ranges) {
    //         std::cout << element << " ";
    //     }
    //     std::cout << std::endl;

    // std::cout << "/////////////////////////////////////////////////////////////" << std::endl;

    // std::vector<float> scanPosDirection(laserScan.ranges.begin(), laserScan.ranges.begin() + scanIndex);
    // std::vector<float> scanNegDirection(laserScan.ranges.end() - scanIndex, laserScan.ranges.end());


    std::vector<std::pair<float, int>> scanPosDirection;
    std::vector<std::pair<float, int>> scanNegDirection;

    // Populate scanPosDirection with range data and indices
    for (int i = 0; i < scanIndex; ++i) {
        scanPosDirection.push_back(std::make_pair(laserScan.ranges[i], i));
    }

    // Populate scanNegDirection with range data and indices
    for (int i = laserScan.ranges.size() - scanIndex; i < laserScan.ranges.size(); ++i) {
        scanNegDirection.push_back(std::make_pair(laserScan.ranges[i], i));
    }



    // Combine the two vectors
    std::vector<std::pair<float, int>> combinedVector = scanNegDirection;
    for (const auto& element : scanPosDirection) {
        combinedVector.push_back(element);
    }

    // // Display the combined vector
    // std::cout << "Combined Vector: ";
    // for (const auto& element : combinedVector) {
    //     std::cout << "(" << element.first << ", " << element.second << ") ";
    // }
    // std::cout << std::endl;

    return combinedVector;

}




// ADD AR TAG REC