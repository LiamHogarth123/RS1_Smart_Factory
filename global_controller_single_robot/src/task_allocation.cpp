// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "task_allocation.hpp"



Task_Allocation::Task_Allocation() {
    job_list_file_path = "/home/liam/git/RS1_Smart_Factory/global_controller_single_robot/joblist.txt";

}


std::vector<geometry_msgs::msg::Point> Task_Allocation::get_job_list(){



    // For testing below

    std::vector<geometry_msgs::msg::Point> job_list;

    // Define five different goals (x, y, z)
    std::vector<std::tuple<double, double, double>> goals = {
        {2.0, 0.0, 0.0},
        {0.0, -2.0, 0.0},
        {-2.0, 0.5, 0.0},
        {0.5, 0, 0.0},
        {0.5, -1.0, 0.0}
    };

    // Fill the job list with points
    for (const auto& goal : goals) {
        geometry_msgs::msg::Point point;
        point.x = std::get<0>(goal);
        point.y = std::get<1>(goal);
        point.z = std::get<2>(goal);

        job_list.push_back(point);
    }

    return job_list;
    

}



void Task_Allocation::Load_job_list_txt(){
    // std::string filename = "/home/liam/git/RS1_Smart_Factory/global_controller_single_robot/joblist.txt";
    std::vector<turtlebot_job> packages;
    std::ifstream file(job_list_file_path);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        turtlebot_job pkg;
        char comma;
        iss >> pkg.id >> comma >> pkg.package_location.x >> comma >> pkg.package_location.y >> comma >> pkg.delivery_location.x >> comma >> pkg.delivery_location.y;
        packages.push_back(pkg);
        turtlebot_job_list = packages;
    }

    std::cout << "loaded job" << std::endl;

    for (int i = 0; i < packages.size(); i++){
        std::cout << packages.at(i).id << std::endl;
    }

}

std::vector<std::vector<turtlebot_job>> Task_Allocation::optimise_turtlebot_jobs(int num_robot,  std::vector<geometry_msgs::msg::Point>& robot_positions){
    Load_job_list_txt();

     std::vector<turtlebot_job> job_list = turtlebot_job_list;
    int num_robots = robot_positions.size();
    std::vector<std::vector<turtlebot_job>> robot_jobs(num_robots);
 
    // Define a single hardcoded delivery location
    geometry_msgs::msg::Point delivery_location;
    delivery_location.x = 5.0;
    delivery_location.y = 5.0;
    delivery_location.z = 0.0;
 
    // Initialize each robot's location
    std::vector<geometry_msgs::msg::Point> current_locations = robot_positions;
 
    // Round-robin assignment of jobs to robots
    int robot_index = 0;
    while (!job_list.empty()) {
        // Step 1: Find the closest initial goal from the current location of the selected robot
        auto nearest_job = std::min_element(job_list.begin(), job_list.end(),
            [&](const turtlebot_job& a, const turtlebot_job& b) {
                return calculateEuclideanDistance(current_locations[robot_index], a.package_location) < calculateEuclideanDistance(current_locations[robot_index], b.package_location);
            });
 
        // Add the nearest job to the selected robot's assigned job list
        robot_jobs[robot_index].push_back(*nearest_job);
 
        // Step 2: Move the robot to the delivery location after pickup
        current_locations[robot_index] = nearest_job->delivery_location;
 
        // Step 3: Remove the job from the list and move to the next robot
        job_list.erase(nearest_job);
        robot_index = (robot_index + 1) % num_robots;
    }
 
    // // Ensure each robot ends at the delivery location
    // for (int i = 0; i < num_robots; ++i) {
    //     // Add a "final delivery" stop for each robot
    //     turtlebot_job end_job;
    //     end_job.id = -1;  // -1 to indicate it's a final stop rather than a specific pickup job
    //     end_job.package_location = delivery_location;  // Set the location to the delivery location
    //     robot_jobs[i].push_back(end_job);
    // }
 
    // Print the job assignments with delivery visits
    for (int i = 0; i < num_robots; ++i) {
        std::cout << "\nRobot " << i + 1 << " is assigned the following jobs:\n";
        for (const auto& job : robot_jobs[i]) {
            if (job.id == -1) {
                std::cout << " - Final stop at delivery location (" << delivery_location.x << ", "
                          << delivery_location.y << ", " << delivery_location.z << ")\n";
            } else {
                std::cout << " - Pick up Job ID: " << job.id << " at (" << job.package_location.x << ", "
                          << job.package_location.y << ", " << job.package_location.z << ")\n";
                std::cout << "   Deliver to (" << delivery_location.x << ", "
                          << delivery_location.y << ", " << delivery_location.z << ")\n";
            }
        }
    }
    return robot_jobs;
 
}



    // calculate Euclidean distance between two points
double Task_Allocation::calculateEuclideanDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b){
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}



std::vector<turtlebot_job> Task_Allocation::PackageSort(const geometry_msgs::msg::Point& current_location) {
    // Get the job list (package pickup locations)
    std::vector<turtlebot_job> job_list = turtlebot_job_list;
    std::vector<turtlebot_job> sorted_jobs;

    // Start from the robot's initial location
    geometry_msgs::msg::Point robot_location = current_location;

    // Greedy algorithm to sort jobs based on the closest package location
    while (!job_list.empty()) {
        // Find the nearest job based on the current robot location
        auto nearest_job = std::min_element(job_list.begin(), job_list.end(),
            [&](const turtlebot_job& a, const turtlebot_job& b) {
                return calculateEuclideanDistance(robot_location, a.package_location) < calculateEuclideanDistance(robot_location, b.package_location);
            });

        // Add the nearest job to the sorted list
        sorted_jobs.push_back(*nearest_job);

        // Update the robot's location to the package pickup location of the nearest job
        robot_location = nearest_job->package_location;

        // Remove the nearest job from the job list (once it's been added to the plan)
        job_list.erase(nearest_job);
    }

    // Define the hardcoded final drop-off location, which is visited last
    turtlebot_job final_drop_off;
    final_drop_off.id = sorted_jobs.size();  // Assign the next available ID
    final_drop_off.package_location.x = 5.0;  // Hardcoded final drop-off (x, y, z)
    final_drop_off.package_location.y = 5.0;
    final_drop_off.package_location.z = 0.0;

    // Add the final drop-off to the sorted job list
    sorted_jobs.push_back(final_drop_off);

    return sorted_jobs;
}


std::vector<turtlebot_job> Task_Allocation::get_Job_List(std::vector<nav_msgs::msg::Odometry> turtlebot_starts) {
    std::vector<turtlebot_job> job_list;
    Load_job_list_txt();
    job_list = PackageSort(turtlebot_starts.at(0).pose.pose.position);
    return job_list;
}

