#include "rclcpp/rclcpp.hpp"
#include "prm_creation.hpp"
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Default namespace is empty
    std::string namespace_param = "";
    if (argc > 1) {
        namespace_param = argv[1];
    }

    // Correct class name and usage
    auto global_controller = std::make_shared<prm_creation>(namespace_param);

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
