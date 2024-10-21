// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "task_allocation.hpp"



Task_Allocation::Task_Allocation() {

}


std::vector<geometry_msgs::msg::Point> Task_Allocation::get_job_list(){



    // For testing below

    std::vector<geometry_msgs::msg::Point> job_list;

    // Define five different goals (x, y, z)
    std::vector<std::tuple<double, double, double>> goals = {
        {1.6, 2.0, 0.0},
        {4.0, 1.0, 0.0},
        {3.0, -1.0, 0.0},
        {4.0, 0, 0.0},
        {2.5, 0.5, 0.0}
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
    std::string filename = "/home/liam/git/RS1_Smart_Factory/global_controller_single_robot/joblist.txt";
    std::vector<Package> packages;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Package pkg;
        char comma;
        iss >> pkg.id >> comma >> pkg.start_x >> comma >> pkg.start_y >> comma >> pkg.finish_x >> comma >> pkg.finish_y;
        packages.push_back(pkg);
        packages_info = packages;
    }

}

std::vector<std::vector<turtlebot_job>> Task_Allocation::Tsp_sort(){
    std::vector<std::vector<turtlebot_job>> Job_Outline;


    //Complete sort. vector for each robot which each robot has a vector of turtlebot_jobs - (id, pickup_location, dropoff_location,)
    // sort packages
    return Job_Outline;
}


