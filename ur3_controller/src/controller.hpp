#ifndef UR3_CONTROLLER_HPP
#define UR3_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class UR3Controller : public rclcpp::Node {
public:
    UR3Controller();
    void control_loop();

    // Function to initialize MoveGroupInterface
    void setupMoveGroup(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group);

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    bool move_to_position(double x, double y, double z);
};

#endif // UR3_CONTROLLER_HPP
