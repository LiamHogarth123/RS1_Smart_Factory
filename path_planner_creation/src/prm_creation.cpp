// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "prm_creation.hpp"
#include <cmath>
#include <std_msgs/msg/bool.hpp>

prm_creation::prm_creation(const std::string &namespace_param) : Node("global_controller") {

    robot_status = false;

   
    // std::string path_topic = "trajectory"; 
    

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&prm_creation::mapCallback, this, std::placeholders::_1));

    map_data_recieved = false;
    


}


void prm_creation::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received a map with resolution: %f", msg->info.resolution);
        RCLCPP_INFO(this->get_logger(), "Map width: %d, height: %d", msg->info.width, msg->info.height);
        map = *msg;
        map_data_recieved = true;
        
}






void prm_creation::Default_state(){
    


    std::vector<geometry_msgs::msg::Point> trajectory;
    rclcpp::Rate rate(10); // 10 Hz
    std_msgs::msg::Bool msg;
    int r = 0;
    while (!map_data_recieved){
        
        if (r < 1){
            std::cout << "first waiting" << std::endl;
            r++;
        }
        rate.sleep();
    }



    // GPS.UpdateMapData(map, map_meta_data);
    
    GPS.UpdateMapData(map, map.info);
    GPS.Generate_Map();
    

    rclcpp::shutdown();

}




























