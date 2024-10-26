#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

class Controller : public rclcpp::Node {
public:
    Controller();
    void set_default_state();
    void control_loop();

private:
    void package_callback(const geometry_msgs::msg::Point::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr package_sub_;
    geometry_msgs::msg::Point package_location_;
    geometry_msgs::msg::Point delivery_location_;
    bool package_received_;
};

#endif // CONTROLLER_HPP