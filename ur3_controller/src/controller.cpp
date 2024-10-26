#include "controller.hpp"

Controller::Controller() : Node("ur3_controller"), package_received_(false) {
    // Initialize the subscription to receive package coordinates
    package_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/package_coordinates", 10,
        std::bind(&Controller::package_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "UR3 Controller Node has started.");
    set_default_state();
}

void Controller::set_default_state() {
    RCLCPP_INFO(this->get_logger(), "Setting UR3 to default state.");
    // Insert code here to initialize the robot's default position
    // Possibly set joint positions to a home position
}

void Controller::package_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received package coordinates: x: %f, y: %f, z: %f", msg->x, msg->y, msg->z);
    package_location_ = *msg;
    package_received_ = true;
    control_loop();
}

void Controller::control_loop() {
    if (!package_received_) {
        RCLCPP_WARN(this->get_logger(), "No package coordinates received. Waiting...");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting control loop for UR3 arm to move to package location.");

    // Move to package location (implement control logic here)
    // Example: Move UR3 to package_location_

    RCLCPP_INFO(this->get_logger(), "Moving to package location: x: %f, y: %f, z: %f", package_location_.x, package_location_.y, package_location_.z);

    // Simulate picking up package and moving to delivery location
    // Define delivery_location_ appropriately for this demo
    delivery_location_ = geometry_msgs::msg::Point();
    delivery_location_.x = 1.0;
    delivery_location_.y = 1.0;
    delivery_location_.z = 1.0;

    RCLCPP_INFO(this->get_logger(), "Moving to delivery location: x: %f, y: %f, z: %f", delivery_location_.x, delivery_location_.y, delivery_location_.z);

    // Reset package_received_ flag to false for next activation
    package_received_ = false;
    RCLCPP_INFO(this->get_logger(), "Package delivered. Control loop complete.");
}