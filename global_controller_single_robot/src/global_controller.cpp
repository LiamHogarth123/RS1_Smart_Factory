// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "global_controller.hpp"
#include <cmath>
#include <std_msgs/msg/bool.hpp>

Global_Controller::Global_Controller(const int &num_robots) : Node("global_controller") {
    robot_status = false;
   
   
    // std::string path_topic = "trajectory"; 
    robot_status_sub = this->create_subscription<std_msgs::msg::Bool>("reached_goal", 10, std::bind(&Global_Controller::statusCallback, this, std::placeholders::_1));\
    Shut_down_request = this->create_publisher<std_msgs::msg::Bool>("shut_down", 10);
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&Global_Controller::mapCallback, this, std::placeholders::_1));
    map_data_recieved = false;

    // this->declare_parameter<int>("number_of_robots");

    // if (this->get_parameter("number_of_robots", number_of_robots_)) {
    //     RCLCPP_INFO(this->get_logger(), "Number of robots set to: %d", number_of_robots_);
    // } else {
    //     RCLCPP_WARN(this->get_logger(), "Parameter 'number_of_robots' not set, using default value.");
    //     number_of_robots_ = 1;  // Optionally set a default value in case it's not set
    // }

}







void Global_Controller::Default_state() {
    std::vector<geometry_msgs::msg::Point> trajectory;
    rclcpp::Rate rate(10); // 10 Hz
    std_msgs::msg::Bool msg;
    int r = 0;
    
    std::cout << "Creating TurtleBot manager..." << std::endl;
    auto manager = std::make_shared<TurtleBotManager>("bacon_bot", 1);
    std::thread spin_thread([manager]() {rclcpp::spin(manager); });
    // Waiting for map data to be received
    while (!map_data_recieved) {
        rate.sleep();
        std::cout << "Waiting for map data..." << std::endl;
    }

 



    // Once map data is received, initialize the path planning system
    std::cout << "Map data received. Initializing path planning system..." << std::endl;
    GPS.UpdateMapData(map);

    // // Create the TurtleBot manager
    // std::cout << "Creating TurtleBot manager..." << std::endl;
    // auto manager = std::make_shared<TurtleBotManager>("");

    // Get the job list (goals)
    std::vector<geometry_msgs::msg::Point> goals = TA.get_job_list();
    std::cout << "Number of goals retrieved: " << goals.size() << std::endl;

    // Print the retrieved goals
    for (size_t i = 0; i < goals.size(); ++i) {
        std::cout << "Goal " << i + 1 << ": (" << goals[i].x << ", " << goals[i].y << ")" << std::endl;
    }



    // Initialize the delivery location and start point
    geometry_msgs::msg::Point delievery_Location1;
    delievery_Location1.x = 0;
    delievery_Location1.y = 2;

    geometry_msgs::msg::Point start;
    start = manager->GetCurrentOdom().pose.pose.position;
    std::cout << "Starting position: (" << start.x << ", " << start.y << ")" << std::endl;

    int package_id = 5;


    for (int i = 0; i < goals.size(); i++){

        trajectory.clear();
        trajectory = GPS.A_star_To_Goal(start, goals.at(i));
        manager->publishTrajectory(trajectory);
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        std::cout << "geeting status" << std::endl;
        std::cout << manager->get_status_bool() << std::endl;
        while (!manager->get_status_bool()){ // for multiple robot the below loop will run on a seperate thread. if synochous
            rate.sleep();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            //Add listener for e-stop
        }

        Process_Package_AR_info(manager->GetARTag(), package_id);
        if (true){ // if (get_AR_Tag = true){
            trajectory.clear();
            trajectory = GPS.A_star_To_Goal(manager->GetCurrentOdom().pose.pose.position, delievery_Location1);
         
            manager->publishTrajectory(trajectory);

            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            std::cout << "geeting status" << std::endl;
            std::cout << manager->get_status_bool() << std::endl;
            
            while (!manager->get_status_bool()){ // for multiple robot the below loop will run on a seperate thread. if synochous
                rate.sleep();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                //Add listener for e-stop
            }       
        

        }
        else {
            // maybe re -  organise goals with task allocation
        }
        



    }


    
    
    msg.data = true;  // Set the boolean value to true
    Shut_down_request->publish(msg); 

    rclcpp::shutdown();

}


bool Global_Controller::Process_Package_AR_info(int Found_tag, int expected_tag){
    // if (Found_tag == expected_tag){
    //     //log package is in transit
    //     return true;
    // }
    // else {
    //     //report missing package
    // }


    return true;
}
















// Note
// Need to communicate robot's odom, status, AR info,
// nav_msgs::msgs::Odometry, int or bool, AR info








// Ros communication functions
////////////////////////////////////////////////////////////////////////////////////////

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



