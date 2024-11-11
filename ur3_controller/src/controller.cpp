#include "controller.hpp"

UR3Controller::UR3Controller()
: Node("ur3_controller") {
    RCLCPP_INFO(this->get_logger(), "UR3 Controller Node Initialized");
}

void UR3Controller::setupMoveGroup(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group) {
    move_group_interface = move_group;
}

bool UR3Controller::move_to_position(double x, double y, double z) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.w = 1.0;  // Facing direction

    move_group_interface->setPoseTarget(target_pose);

    auto success = move_group_interface->move();
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Moved to position (%.2f, %.2f, %.2f)", x, y, z);
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to move to position (%.2f, %.2f, %.2f)", x, y, z);
        return false;
    }
}

void UR3Controller::control_loop() {
    std::vector<std::array<double, 3>> positions = {
        {0.3, 0.2, 0.5},  // Pickup position
        {0.6, -0.2, 0.3}  // Drop-off position
    };

    while (rclcpp::ok()) {
        for (const auto& pos : positions) {
            if (!move_to_position(pos[0], pos[1], pos[2])) {
                RCLCPP_ERROR(this->get_logger(), "Movement failed, aborting loop");
                return;
            }
            rclcpp::sleep_for(std::chrono::seconds(2));  // Pause to simulate pickup/drop-off
        }
    }
}
