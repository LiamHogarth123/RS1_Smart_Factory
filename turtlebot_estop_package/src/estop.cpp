#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class EstopController {
public:
    EstopController(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node), is_sending_zero_velocity_(false) {
        // Initialize the velocity publisher
        velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    void default_state() {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        
        stop_msg.angular.z = 0.0;

        RCLCPP_INFO(node_->get_logger(), "E-Stop activated. Press Space to start zero-velocity, X to stop, Escape to exit.");

        char key;
        while (rclcpp::ok()) {
            key = getKeyPress();

            if (key == 32) { // Space bar ASCII code
                RCLCPP_INFO(node_->get_logger(), "Space pressed - Sending zero velocity commands...");
                is_sending_zero_velocity_ = true;
            } else if (key == 'x' || key == 'X') {
                RCLCPP_INFO(node_->get_logger(), "X pressed - Stopping zero velocity commands...");
                is_sending_zero_velocity_ = false;
            } else if (key == 27) { // Escape ASCII code
                RCLCPP_INFO(node_->get_logger(), "Escape pressed - Exiting E-Stop mode.");
                break;
            }

            // Publish zero velocity if enabled
            if (is_sending_zero_velocity_) {
                std::cout << "publishing zero velocity" << std::endl;
                velocity_publisher_->publish(stop_msg);
                velocity_publisher_->publish(stop_msg);
                velocity_publisher_->publish(stop_msg);
                velocity_publisher_->publish(stop_msg);
                velocity_publisher_->publish(stop_msg);


            }

         
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Adjust frequency as needed
        }
        rclcpp::spin_some(node_);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    bool is_sending_zero_velocity_;

    char getKeyPress() {
        // Capture a single keypress
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }
};