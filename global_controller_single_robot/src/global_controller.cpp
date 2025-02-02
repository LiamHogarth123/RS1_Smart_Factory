// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "global_controller.hpp"
#include <cmath>
#include <std_msgs/msg/bool.hpp>

Global_Controller::Global_Controller(const int &num_robots) : Node("global_controller") {
    robot_status = false;
   
   
    // std::string path_topic = "trajectory"; 
    robot_status_sub = this->create_subscription<std_msgs::msg::Bool>("reached_goal", 10, std::bind(&Global_Controller::statusCallback, this, std::placeholders::_1));
    start_subscription_ = this->create_subscription<std_msgs::msg::Bool>("start", 10, std::bind(&Global_Controller::startCallback, this, std::placeholders::_1));
        
    Shut_down_request = this->create_publisher<std_msgs::msg::Bool>("shut_down", 10);
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&Global_Controller::mapCallback, this, std::placeholders::_1));
    map_data_recieved = false;


    publisher_ = this->create_publisher<std_msgs::msg::Int32>("package_status", 10);

    // this->declare_parameter<int>("number_of_robots");

    // if (this->get_parameter("number_of_robots", number_of_robots_)) {
    //     RCLCPP_INFO(this->get_logger(), "Number of robots set to: %d", number_of_robots_);
    // } else {
    //     RCLCPP_WARN(this->get_logger(), "Parameter 'number_of_robots' not set, using default value.");
    //     number_of_robots_ = 1;  // Optionally set a default value in case it's not set
    // }

}



void Global_Controller::startCallback(const std_msgs::msg::Bool::SharedPtr msg){
    if (msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "Received start signal: TRUE");
        // Add logic to handle the start signal
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Received start signal: FALSE");
        // Add logic for stop or other behavior
    }
}





void Global_Controller::Default_state() {
    std::vector<geometry_msgs::msg::Point> trajectory;
    rclcpp::Rate rate(10); // 10 Hz
    std_msgs::msg::Bool msg;
    int r = 0;
    
    std::cout << "Creating TurtleBot manager..." << std::endl;
    auto manager = std::make_shared<TurtleBotManager>("", 1);
    std::thread spin_thread([manager]() {rclcpp::spin(manager); });
    // Waiting for map data to be received

    std::cout << "Waiting for map data..." << std::endl;


    while (!map_data_recieved) {
        rate.sleep();
    }

 



    // Once map data is received, initialize the path planning system
    std::cout << "Map data received. Initializing path planning system..." << std::endl;
    GPS.UpdateMapData(map);

    // Get the job list (goals)
    // std::vector<geometry_msgs::msg::Point> goals = TA.get_job_list();
    // std::cout << "Number of goals retrieved: " << goals.size() << std::endl;
    std::vector<nav_msgs::msg::Odometry> turtlebot_start_odom;
    turtlebot_start_odom.push_back(manager->GetCurrentOdom());
    std::vector<turtlebot_job> job_list = TA.get_Job_List(turtlebot_start_odom);



    for (int i = 0; i < job_list.size(); i++){
        std::cout << "package x " << job_list.at(i).package_location.x << "y" << job_list.at(i).package_location.y << std::endl; 
        std::cout << "deliever x " << job_list.at(i).delivery_location.x << "y" << job_list.at(i).delivery_location.y << std::endl;
    }


    



    geometry_msgs::msg::Point start;
    start = manager->GetCurrentOdom().pose.pose.position;
    std::cout << "Starting position: (" << start.x << ", " << start.y << ")" << std::endl;

    int package_id = 5;


    for (int i = 0; i < job_list.size(); i++){

        trajectory.clear();
        trajectory = GPS.A_star_To_Goal(manager->GetCurrentOdom().pose.pose.position, job_list.at(i).package_location);
        manager->publishTrajectory(trajectory);

        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        std::cout << "geeting status" << std::endl;
        std::cout << manager->get_status_bool() << std::endl;
        
        while (!manager->get_status_bool()){ // for multiple robot the below loop will run on a seperate thread. if synochous
            rate.sleep();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            //Add listener for e-stop
        }

        bool correct_package_found = Process_Package_AR_info(manager->GetARTag(), job_list.at(i).id);
        
        if (correct_package_found){ // if (get_AR_Tag = true){
            trajectory.clear();
            trajectory = GPS.A_star_To_Goal(manager->GetCurrentOdom().pose.pose.position, job_list.at(i).delivery_location);
         
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
            // maybe re -  organise goals with task allocation if package not found
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




void Global_Controller::publish_package_status(int package_id, int status_code){

    if (package_id < 1 || package_id > 8) {
        RCLCPP_ERROR(this->get_logger(), "Invalid package ID: %d", package_id);
        return;
    }
    if (status_code != 1 && status_code != 2 && status_code != 3 &&
        status_code != 4 && status_code != 5 && status_code != 6 &&
        status_code != 8 && status_code != 9) {
        RCLCPP_ERROR(this->get_logger(), "Invalid status code: %d", status_code);
        return;
    }

    // Combine package ID and status code into a single number
    auto message = std_msgs::msg::Int32();
    message.data = package_id * 10 + status_code;

    // Publish the message
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: Package ID %d, Status %d", package_id, status_code);

}




void Global_Controller::Default_state_multi() {
    std::vector<geometry_msgs::msg::Point> trajectory;
    rclcpp::Rate rate(10); // 10 Hz
    std_msgs::msg::Bool msg;
    std::cout << "Waiting for map data..." << std::endl;

    
    while (!map_data_recieved) {
        rate.sleep();
        std::cout << "Waiting for map data..." << std::endl;

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(40000));  // Small delay to prevent busy-waiting


    
    int r = 0;
    
    int num_robots = 4;

    // std::cout << "Creating TurtleBot manager..." << std::endl;
    // auto manager = std::make_shared<TurtleBotManager>("bacon_bot", 1);
    // std::thread spin_thread([manager]() {rclcpp::spin(manager); });
    // // Waiting for map data to be received


    std::vector<std::shared_ptr<TurtleBotManager>> managers;
    std::vector<std::thread> spin_threads;

    std::cout << "Creating TurtleBot managers..." << std::endl;

    for (int i = 1; i <= num_robots; ++i) {
        // Create the namespace as "tb" followed by the robot number, e.g., tb1, tb2, etc.
        std::string namespace_name = "tb" + std::to_string(i);

        // Instantiate a TurtleBotManager with the namespace and ID
        auto manager = std::make_shared<TurtleBotManager>(namespace_name, i);

        // Add the manager to the vector to keep it alive
        managers.push_back(manager);

        // Start a separate thread to spin each manager
        spin_threads.emplace_back([manager]() { rclcpp::spin(manager); });
    }

    std::vector<geometry_msgs::msg::Point> turtlebot_start;

    for (int i = 0; i < managers.size(); i++){
        turtlebot_start.push_back(managers.at(i)->GetCurrentOdom().pose.pose.position);
    }

 



    // Once map data is received, initialize the path planning system
    std::cout << "Map data received. Initializing path planning system..." << std::endl;
    GPS.UpdateMapData(map);


    std::cout << "Path planner initialised, run task allocation" << std::endl;
    // std::vector<turtlebot_job> job_list = 




    // for (int i = 0; i < job_list.size(); i++){
    //     std::cout << "package x " << job_list.at(i).package_location.x << "y" << job_list.at(i).package_location.y << std::endl; 
    //     std::cout << "deliever x " << job_list.at(i).delivery_location.x << "y" << job_list.at(i).delivery_location.y << std::endl;
    // }


    


    std::vector<std::vector<geometry_msgs::msg::Point>> trajectory_points;
    std::vector<std::vector<turtlebot_job>> multi_turtlebot_job_list = TA.optimise_turtlebot_jobs(num_robots, turtlebot_start);

    std::cout << "checking job list size" << std::endl;
    std::cout << multi_turtlebot_job_list.size() << std::endl;
    std::cout << multi_turtlebot_job_list.at(0).size() << std::endl;
    
    for (int job_list_idex = 0; job_list_idex < multi_turtlebot_job_list.size(); job_list_idex++){
        trajectory_points.clear();

        for (int robot_index = 0; robot_index < num_robots; robot_index++){    
            trajectory_points.push_back(GPS.A_star_To_Goal(managers.at(robot_index)->GetCurrentOdom().pose.pose.position, multi_turtlebot_job_list.at(robot_index).at(job_list_idex).package_location));
        }

        for(int publishing_index = 0; publishing_index < trajectory_points.size(); publishing_index++){
            managers.at(publishing_index)->publishTrajectory(trajectory_points.at(publishing_index));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20000));  // Small delay to prevent busy-waiting



        bool all_true = false;
        while (!all_true) {
            all_true = std::all_of(managers.begin(), managers.end(),
                            [](const std::shared_ptr<TurtleBotManager>& manager) {
                                return manager->get_status_bool();
                            });
            if (!all_true) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Small delay to prevent busy-waiting
            }
        }


        //check all AR tags
        // bool correct_package_found = Process_Package_AR_info(manager->GetARTag(), job_list.at(i).id);
        

        if (true){ // if (get_AR_Tag = true){

            trajectory_points.clear();

            for (int robot_index = 0; robot_index < num_robots; robot_index++){    
                trajectory_points.push_back(GPS.A_star_To_Goal(managers.at(robot_index)->GetCurrentOdom().pose.pose.position, multi_turtlebot_job_list.at(job_list_idex).at(robot_index).delivery_location));
            }

            for(int publishing_index = 0; publishing_index < trajectory_points.size(); publishing_index++){
                managers.at(publishing_index)->publishTrajectory(trajectory_points.at(publishing_index));
            }



            bool all_true = false;
            while (!all_true) {
                all_true = std::all_of(managers.begin(), managers.end(),
                                [](const std::shared_ptr<TurtleBotManager>& manager) {
                                    return manager->get_status_bool();
                                });
                if (!all_true) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Small delay to prevent busy-waiting
                }

            }       

        }
        else {
            // maybe re -  organise goals with task allocation if package not found
        }

    }
    
    
    msg.data = true;  // Set the boolean value to true
    Shut_down_request->publish(msg); 

    rclcpp::shutdown();

}







void Global_Controller::Default_state_multi_complex() {
    std::vector<geometry_msgs::msg::Point> trajectory;
    rclcpp::Rate rate(10); // 10 Hz
    std_msgs::msg::Bool msg;
    std::cout << "Waiting for map data..." << std::endl;

    
    while (!map_data_recieved) {
        rate.sleep();
        std::cout << "Waiting for map data..." << std::endl;

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(40000));  // Small delay to prevent busy-waiting


    
    int r = 0;
    
    int num_robots = 4;

    // std::cout << "Creating TurtleBot manager..." << std::endl;
    // auto manager = std::make_shared<TurtleBotManager>("bacon_bot", 1);
    // std::thread spin_thread([manager]() {rclcpp::spin(manager); });
    // // Waiting for map data to be received


    std::vector<std::shared_ptr<TurtleBotManager>> managers;
    std::vector<std::thread> spin_threads;

    std::cout << "Creating TurtleBot managers..." << std::endl;

    for (int i = 1; i <= num_robots; ++i) {
        // Create the namespace as "tb" followed by the robot number, e.g., tb1, tb2, etc.
        std::string namespace_name = "tb" + std::to_string(i);

        // Instantiate a TurtleBotManager with the namespace and ID
        auto manager = std::make_shared<TurtleBotManager>(namespace_name, i);

        // Add the manager to the vector to keep it alive
        managers.push_back(manager);

        // Start a separate thread to spin each manager
        spin_threads.emplace_back([manager]() { rclcpp::spin(manager); });
    }

    std::vector<geometry_msgs::msg::Point> turtlebot_start;

    for (int i = 0; i < managers.size(); i++){
        turtlebot_start.push_back(managers.at(i)->GetCurrentOdom().pose.pose.position);
    }

 



    // Once map data is received, initialize the path planning system
    std::cout << "Map data received. Initializing path planning system..." << std::endl;
    GPS.UpdateMapData(map);


    std::cout << "Path planner initialised, run task allocation" << std::endl;
    // std::vector<turtlebot_job> job_list = 

    std::vector<std::vector<geometry_msgs::msg::Point>> trajectory_points;
    std::vector<geometry_msgs::msg::Point> single_trajectory;
    std::vector<std::vector<turtlebot_job>> multi_turtlebot_job_list = TA.optimise_turtlebot_jobs(num_robots, turtlebot_start);
    std::vector<int> each_robot_goal_index;
    std::vector<nav_msgs::msg::Odometry> current_odoms;

    for (int j = 0; j < managers.size(); j++){
        each_robot_goal_index.push_back(1);
    }




    trajectory_points.clear();
    

    for (int robot_index = 0; robot_index < num_robots; robot_index++){    
        trajectory_points.push_back(GPS.A_star_To_Goal(managers.at(robot_index)->GetCurrentOdom().pose.pose.position, multi_turtlebot_job_list.at(robot_index).at(0).package_location));
    }

    for(int publishing_index = 0; publishing_index < trajectory_points.size(); publishing_index++){
        publish_package_status(multi_turtlebot_job_list.at(publishing_index).at(0).id, 2);
        managers.at(publishing_index)->publishTrajectory(trajectory_points.at(publishing_index));
        
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20000));  // Small delay to prevent busy-waiting




    std::vector<int> finish_ids;
    bool all_goals_reached = false;
    
    
    while (!all_goals_reached) {
        for (int i = 0; i < managers.size(); i++){
            bool status_check = managers.at(i)->get_status_bool();

            
            // Checks to see if the each robot has finished all its goal and if it needs to be checked.
            if (std::find(finish_ids.begin(), finish_ids.end(), i) != finish_ids.end()) {
                continue; // Skip if the robot is already finished
            }
            else if (each_robot_goal_index.at(i)/2 >= multi_turtlebot_job_list.at(i).size()) {
                finish_ids.push_back(i);
                if (finish_ids.size() == managers.size()){
                    all_goals_reached = true;
                    break;
                }
                continue;
            }
        
            //Then check status of robot to see if it has reach a goal
             if (!status_check) {
                continue; // Skip if the robot has not reached a goal
            }
            // //get odoms of all robots
            // for (int j = 0; j < managers.size(); j++){
            //     current_odoms.at(j) = managers.at(j)->GetCurrentOdom();
            // }



            //Checkecks if index is even or odd to see if the robot needs to drive to delievery or pickup 
            if (each_robot_goal_index.at(i) % 2 == 0){ //if index is even
                
                std::cout << "robot " << i << " heading to goal " << (each_robot_goal_index.at(i)/2) << std::endl;
                std::cout << "package at x" << multi_turtlebot_job_list.at(i).at(each_robot_goal_index.at(i)/2).package_location.x << " y"  << multi_turtlebot_job_list.at(i).at(each_robot_goal_index.at(i)/2).package_location.y << std::endl;
                publish_package_status(multi_turtlebot_job_list.at(i).at(each_robot_goal_index.at(i)/2).id, 2);
               
                
                single_trajectory = GPS.A_star_To_Goal(managers.at(i)->GetCurrentOdom().pose.pose.position, multi_turtlebot_job_list.at(i).at(each_robot_goal_index.at(i)/2).package_location);
                trajectory_points.at(i) = single_trajectory;
                // path_aviodance_checker.check_trajectory_odom(trajectory_points, current_odoms);



                managers.at(i)->publishTrajectory(single_trajectory);
            }
            else { // if index is odd   if (each_robot_goal_index.at(i) % 2 != 0)
                
                std::cout << "robot " << i << " heading to delievery location" << multi_turtlebot_job_list.at(i).at(each_robot_goal_index.at(i)/2).id << std::endl;
                std::cout << "delivery at x" << multi_turtlebot_job_list.at(i).at(each_robot_goal_index.at(i)/2).delivery_location.x << " y" << multi_turtlebot_job_list.at(i).at(each_robot_goal_index.at(i)/2).delivery_location.y << std::endl;
                publish_package_status((multi_turtlebot_job_list.at(i).at((each_robot_goal_index.at(i)-1)/2)).id, 3);


                if (true) { // check AR tag status
                    std::cout << "Ar tag found" << std::endl;

                    //Add trajectory checking planning
                    single_trajectory = GPS.A_star_To_Goal(managers.at(i)->GetCurrentOdom().pose.pose.position, multi_turtlebot_job_list.at(i).at((each_robot_goal_index.at(i)-1)/2).delivery_location);
                    trajectory_points.at(i) = single_trajectory;
                        // path_aviodance_checker.check_trajectory_odom(trajectory_points, current_odoms);
                    managers.at(i)->publishTrajectory(single_trajectory);
                
                }
                else {
                    continue;
                }

            
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));

            // else if (docking){
            //     add docking serario
            // }
            
            
            each_robot_goal_index.at(i) = each_robot_goal_index.at(i) + 1;
        


        }
    }


        //check all AR tags
        // bool correct_package_found = Process_Package_AR_info(manager->GetARTag(), job_list.at(i).id);
        


    std::cout << "Program complete Shutting down" << std::endl;
    
    
    msg.data = true;  // Set the boolean value to true
    Shut_down_request->publish(msg); 

    rclcpp::shutdown();

}