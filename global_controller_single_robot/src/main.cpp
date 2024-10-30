#include "rclcpp/rclcpp.hpp"
#include "global_controller.hpp"
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

 
    int number_of_robot_parameter;

    // Correct class name and usage
    auto global_controller = std::make_shared<Global_Controller>(number_of_robot_parameter);

    // Start the defaultStat function in a separate thread
    std::thread default_stat_thread([&global_controller]() {global_controller->Default_state();});

    // Spin the node to handle incoming messages
    rclcpp::spin(global_controller);

    // Wait for the defaultStat thread to finish before shutting down
    if (default_stat_thread.joinable()) {
        default_stat_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
