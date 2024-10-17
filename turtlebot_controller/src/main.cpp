#include "rclcpp/rclcpp.hpp"
#include "controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Default namespace is empty
    std::string namespace_param = "";
    if (argc > 1) {
        namespace_param = argv[1];
    }
    auto controller_node = std::make_shared<Controller>(namespace_param);

    // Start the defaultStat function in a separate thread
    std::thread default_stat_thread([&controller_node]() {controller_node->Default_state();});


    // Spin the node to handle incoming messages
    rclcpp::spin(controller_node);

    // Wait for the defaultStat thread to finish before shutting down
    if (default_stat_thread.joinable()) {
        default_stat_thread.join();
    }

    rclcpp::shutdown();
}
