// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "global_controller.hpp"
#include <cmath>
#include <std_msgs/msg/bool.hpp>

Global_Controller::Global_Controller(const std::string &namespace_param) : Node("global_controller") {

    robot_status = false;

   
    // std::string path_topic = "trajectory"; 
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
    robot_status_sub = this->create_subscription<std_msgs::msg::Bool>("reached_goal", 10, std::bind(&Global_Controller::statusCallback, this, std::placeholders::_1));\
    Shut_down_request = this->create_publisher<std_msgs::msg::Bool>("shut_down", 10);

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&Global_Controller::mapCallback, this, std::placeholders::_1));

    map_data_recieved = false;
    


}





void Global_Controller::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received a map with resolution: %f", msg->info.resolution);
        RCLCPP_INFO(this->get_logger(), "Map width: %d, height: %d", msg->info.width, msg->info.height);
        map = *msg;
        map_data_recieved = true;
        
}




void Global_Controller::statusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Goal reached, sending next trajectory.");
        robot_status = true;
    }
}




void Global_Controller::Default_state(){
    

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
    
    // GPS.GeneratePRM(map, map.info, true);
    

    

    
    std::vector<geometry_msgs::msg::Point> goals = TA.get_job_list();
    geometry_msgs::msg::Point delievery_Location1;
    delievery_Location1.x = 0;
    delievery_Location1.y = 0;
    geometry_msgs::msg::Point start;
    start.x = -2;
    start.y = 0.5;


    
    
    for (int i = 0; i < goals.size(); i++){
        trajectory.clear();
        trajectory.push_back(goals.at(i)); // convert this line to trajectory = PRM.genarate_path(Robot_odom, goals.at(i))
        // trajectory.clear();
        // trajectory = GPS.A_star_To_Goal(start, goals.at(i));
        publishTrajectory(trajectory);

        int k = 0;
        while (!robot_status){ // for multiple robot the below loop will run on a seperate thread. if synochous
            if (k == 0){
                std::cout << "waiting1" << std::endl;
                k++;
            }
            rate.sleep();

        }
        
        if (true){ // if (get_AR_Tag = true){
            trajectory.clear();
            // trajectory = GPS.A_star_To_Goal(goals.at(i), delievery_Location1);
            trajectory.push_back(delievery_Location1); // convert this line to trajectory = PRM.genarate_path(start, end);
         
            publishTrajectory(trajectory);

            int j = 0;
            while (!robot_status){
                if (j == 0){
                    std::cout << "waiting2" << std::endl;
                    j++;
                }

            rate.sleep();

            }

        }
        // else {
        // report missing package
        // maybe read organise goals with task allocation
        // }
        


    }


    
    
    msg.data = true;  // Set the boolean value to true
    Shut_down_request->publish(msg); 

    rclcpp::shutdown();

}






























void Global_Controller::publishTrajectory(std::vector<geometry_msgs::msg::Point> goals){
    auto path_msg = std::make_shared<nav_msgs::msg::Path>();
    path_msg->header.stamp = this->get_clock()->now();
    path_msg->header.frame_id = "map"; // Replace with your frame of reference

    // Define five different (x, y) coordinates
    

    for (const auto& coord : goals) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map"; // Replace with your frame of reference
        pose.pose.position.x = coord.x;
        pose.pose.position.y = coord.y;
        pose.pose.position.z = 0.0; // Assuming a flat plane

        // You can add orientation if needed, here assuming no orientation
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        // Add the pose to the path
        path_msg->poses.push_back(pose);
    }

    // Publish the path message
    path_pub_->publish(*path_msg);
    robot_status = false;
    RCLCPP_INFO(this->get_logger(), "Published a trajectory with %zu points", path_msg->poses.size());

}



