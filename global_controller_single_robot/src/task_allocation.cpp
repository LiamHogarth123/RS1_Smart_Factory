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

std::vector<std::vector<turtlebot_job>> Task_Allocation::optimise_turtlebot_jobs(int num_robot){
    Load_job_list_txt();

    std::vector<std::vector<turtlebot_job>> sorted_jobs(num_robot);

    // // // Step 1: Initialize jobs
    // // std::vector<turtlebot_job> jobs;
    // // for (const auto& package : turtlebot_job_list) {
    // //     turtlebot_job job{package.id, package.pickup_location, package.delivery_location};
    // //     jobs.push_back(job);
    // // }

    // // Step 2: Assign jobs to each robot based on nearest package location
    // for (int robot_idx = 0; robot_idx < num_robot; ++robot_idx) {
    //     // Find closest package for each TurtleBot and assign
    //     std::sort(turtlebot_job_list.begin(), turtlebot_job_list.end(), [&](const turtlebot_job& a, const turtlebot_job& b) {
    //         geometry_msgs::msg::Point robot_start = getRobotStartPosition(robot_idx); // Define this function
    //         return calculateEuclideanDistance(robot_start, a.package_location) < calculateEuclideanDistance(robot_start, b.package_location);
    //     });

    //     // Step 3: Allocate sorted jobs to this TurtleBot, avoiding overlap
    //     int job_count = turtlebot_job_list.size() / num_robot;
    //     auto start_itr = jobs.begin() + robot_idx * job_count;
    //     auto end_itr = (robot_idx == num_robot - 1) ? jobs.end() : start_itr + job_count;
    //     sorted_jobs[robot_idx].insert(sorted_jobs[robot_idx].end(), start_itr, end_itr);

    //     // Sort jobs for each TurtleBot to minimize travel distance between consecutive jobs
    //     std::sort(sorted_jobs[robot_idx].begin(), sorted_jobs[robot_idx].end(), [&](const turtlebot_job& a, const turtlebot_job& b) {
    //         return calculateEuclideanDistance(a.delivery_location, b.package_location) < calculateEuclideanDistance(a.package_location, b.delivery_location);
    //     });
    // }
    
    return sorted_jobs;
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