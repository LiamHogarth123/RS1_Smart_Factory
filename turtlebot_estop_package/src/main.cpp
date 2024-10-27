/**
 * @file main.cpp
 * @brief Entry point for the Cylinder Detection and Mapping System.
 * 
 * Initializes the ROS 2 node and starts the LIDAR processing system that detects cylinders and marks them on a SLAM map.
 */

#include "rclcpp/rclcpp.hpp"
#include "estop.cpp"

/**
 * @brief Main function that initializes the ROS 2 node and spins.
 * 
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char**argv) {
    rclcpp::init(argc, argv);

    // Create an instance of your node
    auto node = std::make_shared<rclcpp::Node>("laserProcessingNode");

    // Initialize the laser processing system
    EstopController processor(node);
    processor.default_state();

    // Start the ROS spinning loop
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
