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
#include <nlohmann/json.hpp>

// Plath planning to aviod other turtlebots
////////////////////////////////////////////////////////////////


// Initialize the static member variable
// std::vector<cv::Point> Path_Planner::polygonPoints;


Path_Planner::Path_Planner()
{
    map_file_path = "/home/liam/map.pgm";
    saved_nodes_path = "/home/liam/git/RS1_Smart_Factory/path_planner_creation/road_map/file.json";
    save_map_file_path = "/home/liam/Desktop/Proof_of_Import!!!.png";
}


std::vector<int> Path_Planner::findPathAStar(const std::unordered_map<int, Node>& map_nodes, int start_id, int finish_id) {
    // Priority queue for exploring nodes, ordered by f_cost
    using QueueElement = std::pair<double, int>; // (f_cost, node_id)
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> open_set;

    // Maps to store g_cost and parent node
    std::unordered_map<int, double> g_cost; // Cost from start node to current node
    std::unordered_map<int, int> came_from; // Parent node

    // Ensure the start and finish nodes are valid
    if (map_nodes.find(start_id) == map_nodes.end() || map_nodes.find(finish_id) == map_nodes.end()) {
        // std::cerr << "Error: Invalid start or finish node ID." << std::endl;
        return {};
    }

    // Initialize g_cost with infinity
    for (const auto& pair : map_nodes) {
        g_cost[pair.first] = std::numeric_limits<double>::infinity();
    }

    // Start node's g_cost is 0
    g_cost[start_id] = 0.0;

    // Insert start node into the priority queue with f_cost = heuristic
    open_set.emplace(heuristic(map_nodes.at(start_id), map_nodes.at(finish_id)), start_id);
    std::cout << "Starting A* from node " << start_id << " to node " << finish_id << std::endl;

    while (!open_set.empty()) {
        // Get the node with the lowest f_cost
        int current_id = open_set.top().second;
        open_set.pop();
        // std::cout << "Processing node: " << current_id << std::endl;

        // If we've reached the goal, reconstruct the path
        if (current_id == finish_id) {
            // std::cout << "Goal node " << finish_id << " reached." << std::endl;
            std::vector<int> path;
            while (current_id != start_id) {
                path.push_back(current_id);
                current_id = came_from[current_id];
            }
            path.push_back(start_id);
            std::reverse(path.begin(), path.end());
            // std::cout << "Path found: ";
            for (int id : path) {
                std::cout << id << " ";
            }
            std::cout << std::endl;
            return path; // Return the path from start to finish
        }

        // Explore neighbors
        const Node& current_node = map_nodes.at(current_id);
        for (const auto& [neighbor_id, cost] : current_node.edges) {
            double tentative_g_cost = g_cost[current_id] + cost;
            // std::cout << "Checking neighbor " << neighbor_id << " with cost " << cost << std::endl;

            // If this path to neighbor is better, update
            if (tentative_g_cost < g_cost[neighbor_id]) {
                // std::cout << "Updating g_cost and came_from for neighbor " << neighbor_id << std::endl;
                came_from[neighbor_id] = current_id;
                g_cost[neighbor_id] = tentative_g_cost;
                double f_cost = tentative_g_cost + heuristic(map_nodes.at(neighbor_id), map_nodes.at(finish_id));
                open_set.emplace(f_cost, neighbor_id);
            }
        }
    }

    // If we exit the loop, there is no path
    std::cerr << "No path found!" << std::endl;
    return {};


}

std::vector<geometry_msgs::msg::Point> Path_Planner::A_star_To_Goal(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal)
{
    std::vector<int> trajectory_node_ID;

    // Print the start and goal points
    std::cout << "Start Point: (" << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "Goal Point: (" << goal.x << ", " << goal.y << ")" << std::endl;

    int start_Id = setGoalNode(start);

    // Print the start node ID
    std::cout << "Start Node ID: " << start_Id << std::endl;

    int Goal_Id = setGoalNode(goal);

    // Print the goal node ID
    std::cout << "Goal Node ID: " << Goal_Id << std::endl;

    trajectory_node_ID = findPathAStar(nodes_Graph, start_Id, Goal_Id);

    // Print the IDs of the nodes in the trajectory
    std::cout << "Trajectory Node IDs: ";
    for (int x = 0; x < trajectory_node_ID.size(); x++) {
        std::cout << trajectory_node_ID.at(x) << " ";
    }
    std::cout << std::endl;

    std::vector<geometry_msgs::msg::Point> trajectory;

    // Print the conversion of nodes to points
    for (int x = 0; x < trajectory_node_ID.size(); x++) {
        geometry_msgs::msg::Point point = convertNodeToPoint(nodes_Graph.at(trajectory_node_ID.at(x)));
        trajectory.push_back(point);
        std::cout << "Converted Point: (" << point.x << ", " << point.y << ")" << std::endl;
    }

    // Print the final goal point being added to the trajectory
    trajectory.push_back(goal);
    std::cout << "Final Goal Point added to trajectory: (" << goal.x << ", " << goal.y << ")" << std::endl;

    path_id = trajectory_node_ID;
    std::cout << path_id.size() << std::endl;
    show_Prm();

    return trajectory;
}

int Path_Planner::setGoalNode(geometry_msgs::msg::Point goal)
{
    // Convert world coordinates to map coordinates
    geometry_msgs::msg::Point map_point;
    map_point.x = (goal.x - SlamMapData.info.origin.position.x) / SlamMapData.info.resolution;
    map_point.y = (goal.y - SlamMapData.info.origin.position.y) / SlamMapData.info.resolution;

    // Print goal and map coordinates
    std::cout << "Goal coordinates: (" << goal.x << ", " << goal.y << ")" << std::endl;
    std::cout << "Converted map coordinates: (" << map_point.x << ", " << map_point.y << ")" << std::endl;

    int closestNodeId = -1;
    float minDistance = std::numeric_limits<float>::infinity();

    // Iterate through all nodes in the graph to find the closest one
    for (const auto &node : nodes_Graph)
    {
        float dx = node.second.x - map_point.x;
        float dy = node.second.y - map_point.y;
        float distance = sqrt(dx * dx + dy * dy);

        // Print distance from each node to the goal
        // std::cout << "Node ID: " << node.second.id << ", Node coordinates: (" << node.second.x << ", " << node.second.y << ")";
        // std::cout << ", Distance to goal: " << distance << std::endl;

        if (distance < minDistance)
        {
            minDistance = distance;
            closestNodeId = node.second.id;

            // Print updated closest node information
            // std::cout << "Updated closest node ID: " << closestNodeId << ", Distance: " << minDistance << std::endl;
        }
    }

    // Print final closest node ID and its distance
    if (closestNodeId != -1) {
        std::cout << "Final closest node ID: " << closestNodeId << ", Distance: " << minDistance << std::endl;
    } else {
        std::cout << "No valid node found!" << std::endl;
    }

    return closestNodeId;
}


geometry_msgs::msg::Point Path_Planner::convertNodeToPoint(Node temp)
{
    geometry_msgs::msg::Point world_point;

    world_point.x = ((temp.x * SlamMapData.info.resolution) + SlamMapData.info.origin.position.x);
    world_point.y = ((temp.y * SlamMapData.info.resolution) + SlamMapData.info.origin.position.y);

    // std::cout << "point x" << world_point.x << " y --" << world_point.y << std::endl;

    return world_point;
}


//Add function to find ID of node at x,y

// A* heuristic (Euclidean distance)
double Path_Planner::heuristic(const Node& a, const Node& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}


void Path_Planner::import_map(){
    nodes_Graph = load_nodes(saved_nodes_path);


}



void from_json(const json& j, std::pair<int, double>& p) {
    j.at("to_id").get_to(p.first);
    j.at("cost").get_to(p.second);
}


void Path_Planner::from_json(const json& j, Node& node) {
    j.at("id").get_to(node.id);
    j.at("x").get_to(node.x);
    j.at("y").get_to(node.y);
    j.at("edges").get_to(node.edges);
}




// Import the nodes map from a JSON file
std::unordered_map<int, Node> Path_Planner::load_nodes(const std::string& filename) {
    std::ifstream file(filename);
    json j_nodes;
    file >> j_nodes;

    std::unordered_map<int, Node> nodes;
    for (const auto& [key, value] : j_nodes.items()) {
        Node node;
        from_json(value, node); // Explicitly call from_json to deserialize into Node
        nodes[std::stoi(key)] = node;
    }
    return nodes;
}


// Visualisation
//////////////////////////////////////////////////////////
void Path_Planner::show_Prm()
{
    std::cout << "Show PRM Opens" << std::endl;
    cv::Mat MapImage = Load_Map();
    MapImage = visalise_prm(MapImage, nodes_Graph);
    MapImage = visalise_path(MapImage, path_id);
    save_map(MapImage);
    // show_map(MapImage);
}

cv::Mat Path_Planner::visalise_path(cv::Mat mapImage, std::vector<int> path) {
    cv::Scalar Line_colour(0, 0, 255); // BGR value for red
    if (path.size() == 0){
        return mapImage;
    }

    for (int i = 1; i < path.size(); i++){
        cv::Point start(nodes_Graph.at(path.at(i-1)).x, nodes_Graph.at(path.at(i-1)).y);

        cv::Point end(nodes_Graph.at(path.at(i)).x, nodes_Graph.at(path.at(i)).y);

        cv::line(mapImage, start, end, Line_colour, 1); // Draw the edge
    }
    return mapImage;
}




void Path_Planner::save_map(cv::Mat mapImage) {
    cv::imwrite(save_map_file_path, mapImage);
}


cv::Mat Path_Planner::Load_Map() {
    // READ Image
    std::cout << "Reading the map image from '/home/liam/map.pgm'...\n";
    cv::Mat grayscaleMapImage = cv::imread(map_file_path, cv::IMREAD_GRAYSCALE);
    if (grayscaleMapImage.empty())
    {
        std::cerr << "Could not open or find the map image" << std::endl;
        return cv::Mat(); // Return an empty Mat if the image could not be loaded
    }

    std::cout << "Successfully loaded the grayscale map image.\n";

    cv::Mat mapImage;
    cv::cvtColor(grayscaleMapImage, mapImage, cv::COLOR_GRAY2BGR);
    std::cout << "Converted grayscale map image to BGR.\n";

    cv::flip(mapImage, mapImage, 0);
    std::cout << "Flipped the map image vertically.\n";

    return mapImage;
}


cv::Mat Path_Planner::visalise_prm(cv::Mat mapImage, std::unordered_map<int, Node> graph_struct)
{
    int radius = 0;              // Adjust this value as needed for the size of the nodes
    cv::Scalar node_color(255, 0, 0); // BGR value for red
    cv::Scalar edge_color(0, 255, 255); // BGR value for yellow
   

   
    // Step 2: Visualize Edges Separately
    std::cout << "Visualizing Edges" << std::endl;
    for (const auto &[node_id, node] : graph_struct)
    {
        cv::Point center(node.x, node.y); // Center point of the current node

        for (const auto &edge : node.edges)
        {
            int connected_node_id = edge.first;
            const auto &connected_node = graph_struct.at(connected_node_id);

            cv::Point connected_node_center(connected_node.x, connected_node.y); // Center point of the connected node

            // Only draw the edge if both nodes are within image bounds
            if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows &&
                connected_node_center.x >= 0 && connected_node_center.x < mapImage.cols &&
                connected_node_center.y >= 0 && connected_node_center.y < mapImage.rows)
            {
                // std::cout << "Edge from Node (" << node_id << ") to Node (" << connected_node_id << ")" << std::endl;
                cv::line(mapImage, center, connected_node_center, edge_color, 1); // Draw the edge
            }
        }
    }
     // Step 1: Visualize Nodes
    std::cout << "Visualizing Nodes" << std::endl;
    for (int m = 0; m < graph_struct.size(); m++){

    // for (const auto &[node_id, node] : graph_struct)
    // {
        // Draw each node as a circle
        cv::Point center(graph_struct.at(m).x, graph_struct.at(m).y); // Flip y-coordinate
        // std::cout << "Node ID: " << graph_struct.at(m).id << "at location" << m << " at (" << graph_struct.at(m).x << ", " << graph_struct.at(m).y << ")" << std::endl;

        // Only draw the node if it's within image bounds
        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows)
        {
            cv::circle(mapImage, center, radius, node_color, -1); // Draw the node as a filled circle
        }
    
    }


    return mapImage; // Return the updated map image with nodes and edges visualized
}


void Path_Planner::UpdateMapData(nav_msgs::msg::OccupancyGrid map) {
    SlamMapData = map;
    import_map();
}
