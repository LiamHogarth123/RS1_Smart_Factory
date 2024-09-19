// 1. Add start configuration cstart to R(N,E)
// 2. Loop
// 3. Randomly Select New Node c to expand
// 4. Randomly Generate new Node c’ from c
// 5. If edge e from c to c’ is collision-free
// 6. Add (c’, e) to R
// 7. If c’ belongs to endgame region, return path
// 8. Return if stopping criteria is met

#include <vector>
#include <utility>
#include "opencv2/opencv.hpp"
// #include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include "prm2.hpp"
#include <random>
#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_map>
#include <limits>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// Plath planning to aviod other turtlebots
////////////////////////////////////////////////////////////////


// Initialize the static member variable
std::vector<cv::Point> Path_Planner::polygonPoints;


Path_Planner::Path_Planner()
{
    // Import(Saved_map_Graph)
}


std::vector<int> Path_Planner::findPathAStar(const std::vector<Node>& graph, int startId, int targetId)
{
    std::vector<int> path_id;


    return path_id; // Return an empty path if no path is found
}


std::vector<geometry_msgs::msg::Point> Path_Planner::A_star_To_Goal(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal)
{
    std::vector<int> trajectory_node_ID;
    
    int start_Id = setGoalNode(start);

    int Goal_Id = setGoalNode(goal);

    
    trajectory_node_ID = findPathAStar(Graph, start_Id, Goal_Id);
    // std::cout << DijkstraNodes.size() << "---------------------------" << std::endl;
    std::vector<geometry_msgs::msg::Point> trajectory;

    for (int x = 0; x < trajectory_node_ID.size(); x++)
    {
        trajectory.push_back(convertNodeToPoint(Graph.at(trajectory_node_ID.at(x))));
    }
    trajectory.push_back(goal);
    return trajectory;
}





//Add function to find ID of node at x,y
