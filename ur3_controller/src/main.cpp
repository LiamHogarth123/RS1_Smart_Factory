#include "controller.hpp"
#include <rclcpp/rclcpp.hpp>


bool waitForParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& param_name, int retries = 10) {
    for (int i = 0; i < retries; ++i) {
        // Attempt to get the parameter value to ensure it's fully loaded
        std::string param_value;
        if (node->get_parameter(param_name, param_value)) {
            RCLCPP_INFO(node->get_logger(), "Found parameter: %s", param_name.c_str());
            return true;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for parameter: %s", param_name.c_str());
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create the UR3Controller node
    auto node = std::make_shared<UR3Controller>();

    // Wait for both `robot_description` and `robot_description_semantic`
    if (!waitForParameter(node, "robot_description") || !waitForParameter(node, "robot_description_semantic")) {
        RCLCPP_ERROR(node->get_logger(), "Required parameters are not available. Exiting.");
        return -1;
    }

    // Initialize MoveGroupInterface after parameters are available
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur5_arm");
    node->setupMoveGroup(move_group);

    // Start control loop
    node->control_loop();

    rclcpp::shutdown();
    return 0;
}