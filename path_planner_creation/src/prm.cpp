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
#include "prm.hpp"
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

bool PRM::checkCollision(const std::vector<geometry_msgs::msg::Point> &traj1, const std::vector<geometry_msgs::msg::Point> &traj2)
{
    // Ensure both trajectories have the same number of points
    double threshold = 0.45;
    double Max_size;
    if (traj1.size() < traj2.size())
    {
        Max_size = traj2.size();
    }

    for (size_t i = 0; i < Max_size; ++i)
    {
        double distance = calculateDistance(traj1[i], traj2[i]);
        if (distance < threshold)
        {
            return true; // Collision detected
        }
    }
    return false; // No collision
}

// Initialize the static member variable
std::vector<cv::Point> PRM::polygonPoints;

PRM::PRM()
{
    // std::cout << "PRM constructor called" << std::endl;
    LPoints.clear();
}

void PRM::User_remove_Nodes()
{
    std::cout << "begining removing nodes" << std::endl;
    cv::Mat MapImage = Load_Map();
    std::cout << "sucessful load of map" << std::endl;

    MapImage = removeNodes(MapImage, Graph);
}

// User Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void PRM::GeneratePRM(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData MapMetaData_, bool User_controlled)
// {
    
//     UpdateMapData(map, MapMetaData_);
//     std::vector<Node> ProbablityRoadMap;
//     if (User_controlled)
//     {
//         ProbablityRoadMap = samplePointsCV();
//     }
//     else
//     {
//         ProbablityRoadMap = samplePoints();
//     }
//     std::vector<Node> ProbablityRoadMap_ = createNodesAndEdges(ProbablityRoadMap);
//     Graph = ProbablityRoadMap_;
//     cv::Mat mapImage_test = Load_Map();
//     mapImage_test = visalise_prm(mapImage_test, Graph);
//     save_map(mapImage_test);
//     User_remove_Nodes();
// }

std::vector<geometry_msgs::msg::Point> PRM::DijkstraToGoal(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal)
{
    std::vector<int> DijkstraNodes;
    // std::cout << "Dijkstra_Opens-------------" <<std::endl;
    int start_Id = setGoalNode(start);

    int Goal_Id = setGoalNode(goal);

    // std::cout << "start_Id  -->" << start_Id << std::endl;
    //     std::cout << "goal_Id  -->" << Goal_Id << std::endl;
    DijkstraNodes = findPathDijkstra(Graph, start_Id, Goal_Id);
    // std::cout << DijkstraNodes.size() << "---------------------------" << std::endl;
    std::vector<geometry_msgs::msg::Point> trajectory;

    for (int x = 0; x < DijkstraNodes.size(); x++)
    {
        trajectory.push_back(convertNodeToPoint(Graph.at(DijkstraNodes.at(x))));
    }
    return trajectory;
}

std::vector<geometry_msgs::msg::Point> PRM::A_star_To_Goal(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal)
{
    std::vector<int> DijkstraNodes;
    // std::cout << "Dijkstra_Opens-------------" <<std::endl;
    int start_Id = setGoalNode(start);

    int Goal_Id = setGoalNode(goal);

    // std::cout << "start_Id  -->" << start_Id << std::endl;
    // std::cout << "goal_Id  -->" << Goal_Id << std::endl;
    DijkstraNodes = findPathAStar(Graph, start_Id, Goal_Id);
    // std::cout << DijkstraNodes.size() << "---------------------------" << std::endl;
    std::vector<geometry_msgs::msg::Point> trajectory;

    for (int x = 0; x < DijkstraNodes.size(); x++)
    {
        trajectory.push_back(convertNodeToPoint(Graph.at(DijkstraNodes.at(x))));
    }
    trajectory.push_back(goal);
    return trajectory;
}

std::vector<geometry_msgs::msg::Point> PRM::A_star_To_Goal_With_Blacklist(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal, std::vector<geometry_msgs::msg::Point> CollisonPoints)
{
    std::vector<int> pathNodes;
    int start_Id = setGoalNode(start);
    int Goal_Id = setGoalNode(goal);

    std::unordered_set<int> blacklist = GenerateBlackList(CollisonPoints);

    pathNodes = findPathAStarWithBlackList(Graph, start_Id, Goal_Id, blacklist);

    std::vector<geometry_msgs::msg::Point> trajectory;
    for (int x = 0; x < pathNodes.size(); x++)
    {
        trajectory.push_back(convertNodeToPoint(Graph.at(pathNodes.at(x))));
    }
    trajectory.push_back(goal);
    return trajectory;
}

void PRM::UpdateMapData(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData MapMetaData_)
{
    std::cout << "PRM map data openning" << std::endl;
    SlamMapData = map;
    numberOfPoints_ = 10000;
    latestMapMetaData_ = MapMetaData_;
}

std::vector<geometry_msgs::msg::Point> PRM::test()
{

    samplePoints();
    std::vector<Node> ProbablityRoadMap;
    ProbablityRoadMap = samplePoints();
    std::cout << "points generated" << std::endl;
    std::cout << "s" << std::endl;
    std::cout << ProbablityRoadMap.size() << std::endl;
    std::vector<Node> ProbablityRoadMap_ = createNodesAndEdges(ProbablityRoadMap);
    std::vector<int> path;
    path = findPathDijkstra(ProbablityRoadMap_, 2, 10);
    // ProbablityRoadMap_ = rotateNodes(ProbablityRoadMap_, latestMapMetaData_.origin.orientation);
    std::cout << "path size" << path.size() << std::endl;
    // visalise_prm(ProbablityRoadMap_, path);
    std::cout << "start Node -> x = " << ProbablityRoadMap_.at(2).x << " -> Y=" << ProbablityRoadMap_.at(2).y << std::endl;
    std::cout << "end Node -> x = " << ProbablityRoadMap_.at(10).x << " -> Y=" << ProbablityRoadMap_.at(10).y << std::endl;
    std::cout << "Distance between nodes -> " << nodeDistance(ProbablityRoadMap_.at(2), ProbablityRoadMap_.at(10)) << std::endl;
    std::vector<geometry_msgs::msg::Point> trajectory;
    trajectory = ConvertParthToWorld(path, ProbablityRoadMap_);

    return trajectory;
}

void PRM::show_Prm()
{
    std::cout << "Show PRM Opens" << std::endl;
    cv::Mat MapImage = Load_Map();
    MapImage = visalise_prm(MapImage, Graph);
    save_map(MapImage);
    show_map(MapImage);
}

PrmData PRM::ExportPrmData()
{
    PrmData Export;
    Export.Exported_Graph = Graph;
    Export.map = SlamMapData;
    Export.MapMetaData_ = latestMapMetaData_;
    return Export;
}

void PRM::Load_PRM(PrmData Imports)
{
    Graph = Imports.Exported_Graph;
    SlamMapData = Imports.map;
    latestMapMetaData_ = Imports.MapMetaData_;
}

// void setoffset(nav_msgs::odometry start_odom){
// do the math
// }

// Gernating PRM
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Node> PRM::samplePoints()
{
    // graph definition
    Graph.clear();
    int id = 0;

    // Random generators
    std::random_device rd;                                                 // Obtain a random number from hardware
    std::mt19937 eng(rd());                                                // Seed the generator
    std::uniform_real_distribution<> distrX(0, latestMapMetaData_.width);  // Define the range for x
    std::uniform_real_distribution<> distrY(0, latestMapMetaData_.height); // Define the range for y

    int barWidth = 70;
    while (Graph.size() < numberOfPoints_)
    {
        geometry_msgs::msg::Point point;
        point.x = distrX(eng);
        point.y = distrY(eng);

        cv::Point cvPoint(point.x, point.y);

        if (isPointInLShape(cvPoint))
        {
            if (ValidPoint(point))
            {
                Node temp;
                temp.x = point.x;
                temp.y = point.y;
                temp.id = id;
                id++;

                Graph.push_back(temp);

                // std::cout << "Point added to graph: (" << temp.x << ", " << temp.y << ")" << std::endl;
            }
            else
            {
                // std::cout << "Point not valid" << std::endl;
            }
        }
        else
        {
            // std::cout << "Point not in L-shaped area" << std::endl;
        }

        // Debugging: Print current size of the graph
        // std::cout << "Current Graph size: " << Graph.size() << std::endl;

        // Display progress bar
        float progress = (float)Graph.size() / numberOfPoints_;
        std::cout << "[";
        int pos = barWidth * progress;
        for (int i = 0; i < barWidth; ++i)
        {
            if (i < pos)
                std::cout << "=";
            else if (i == pos)
                std::cout << ">";
            else
                std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();
    }
    std::cout << std::endl;

    return Graph;
}

bool PRM::isPointInLShape(const cv::Point &point)
{
    if (LPoints.empty())
    {

        cv::Point A, B, C, D, E, F;

        // Set the coordinates for the corners of the L shape
        A.x = 3;
        A.y = 5.5;
        B.x = -5;
        B.y = 5.5;
        C.x = -5;
        C.y = 1;
        D.x = -0.2;
        D.y = 1;
        E.x = -0.2;
        E.y = -1;
        F.x = 3;
        F.y = -0.75;

        A = convertPointToNodeCordinate(A);
        B = convertPointToNodeCordinate(B);
        C = convertPointToNodeCordinate(C);
        D = convertPointToNodeCordinate(D);
        E = convertPointToNodeCordinate(E);
        F = convertPointToNodeCordinate(F);

        // std::cout << "Vertices of L shape in grid coordinates:" << std::endl;
        // std::cout << "A: (" << A.x << ", " << A.y << ")" << std::endl;
        // std::cout << "B: (" << B.x << ", " << B.y << ")" << std::endl;
        // std::cout << "C: (" << C.x << ", " << C.y << ")" << std::endl;
        // std::cout << "D: (" << D.x << ", " << D.y << ")" << std::endl;
        // std::cout << "E: (" << E.x << ", " << E.y << ")" << std::endl;
        // std::cout << "F: (" << F.x << ", " << F.y << ")" << std::endl;

        LPoints.push_back(A);
        LPoints.push_back(B);
        LPoints.push_back(C);
        LPoints.push_back(D);
        LPoints.push_back(E);
        LPoints.push_back(F);
    }

    bool result = cv::pointPolygonTest(LPoints, point, false) >= 0;
    // std::cout << "isPointInPolygon called with point: (" << point.x << ", " << point.y << "), result: " << result << std::endl;
    return result;
}

bool PRM::ValidPoint(geometry_msgs::msg::Point point)
{

    int grid_x = static_cast<int>(point.x);
    int grid_y = static_cast<int>(point.y);

    int index = grid_x + (grid_y * SlamMapData.info.width);
    int grid_size = 8;

    // Loop through the grid_size x grid_size area around the point
    for (int dx = -grid_size / 2; dx <= grid_size / 2; ++dx)
    {
        for (int dy = -grid_size / 2; dy <= grid_size / 2; ++dy)
        {
            int new_x = grid_x + dx;
            int new_y = grid_y + dy;

            // Check if the new point is within the map boundaries
            if (new_x < 0 || new_x >= SlamMapData.info.width || new_y < 0 || new_y >= SlamMapData.info.height)
            {
                return false; // If the point is outside the map boundaries, return false
            }

            // Calculate the index of the new point
            int new_index = new_x + (new_y * SlamMapData.info.width);

            // Check if the new point is within the map boundaries
            if (new_index >= SlamMapData.data.size() || new_index < 0)
            {
                return false;
            }

            // Check if the point is free of obstacles (value 0)
            if (SlamMapData.data.at(new_index) != 0)
            {
                return false;
            }
        }
    }

    return true; // All points in the grid are free of obstacles
}

bool PRM::pathIsClear(Node Node_A, Node Node_B)
{
    // Generate line between points
    ////////////////////////////////////////////////////////////
    float x1 = Node_A.x;
    float x2 = Node_B.x;
    float y1 = Node_A.y;
    float y2 = Node_B.y;

    std::vector<std::pair<int, int>> BressenhamPoints;

    BressenhamPoints = bresenhamLinePoints(x1, y1, x2, y2);

    // Validating line
    /////////////////////////////////////////////////////
    for (const auto &pair : BressenhamPoints)
    {

        // cv::Point point(pair.first, pair.second);
        // path_points_withoutValidation.push_back(point);
    }

    for (const auto &pair : BressenhamPoints)
    {
        geometry_msgs::msg::Point temp;
        temp.x = pair.first;
        temp.y = pair.second;
        if (ValidPointForPath(pair.first, temp.y))
        {
            // cv::Point point(pair.first, pair.second);
            // path_points.push_back(point);
        }
        else
        {
            // std::cout << "invalid Point " << std::endl;
            return false;
        }
    }

    for (const auto &pair : BressenhamPoints)
    {

        // cv::Point point(pair.first, pair.second-16);
        // path_points.push_back(point);   geometry_msgs
    }

    return true;
}

bool PRM::ValidPointForPath(int x1, int y1)
{

    int grid_x = x1;
    int grid_y = y1;

    // // Calculate index using grid indices
    int index = grid_x + (grid_y * SlamMapData.info.width);

    // Define the size of the grid to check around the point
    int grid_size = 8;

    // Loop through the grid_size x grid_size area around the point
    for (int dx = -grid_size / 2; dx <= grid_size / 2; ++dx)
    {
        for (int dy = -grid_size / 2; dy <= grid_size / 2; ++dy)
        {
            int new_x = grid_x + dx;
            int new_y = grid_y + dy;

            // Check if the new point is within the map boundaries
            if (new_x < 0 || new_x >= SlamMapData.info.width || new_y < 0 || new_y >= SlamMapData.info.height)
            {
                return false; // If the point is outside the map boundaries, return false
            }

            // Calculate the index of the new point
            int new_index = new_x + (new_y * SlamMapData.info.width);

            // Check if the new point is within the map boundaries
            if (new_index >= SlamMapData.data.size() || new_index < 0)
            {
                return false;
            }

            // Check if the point is free of obstacles (value 0)
            if (SlamMapData.data.at(new_index) != 0)
            {
                return false;
            }
        }
    }

    return true; // All points in the grid are free of obstacles
}

std::vector<std::pair<int, int>> PRM::bresenhamLinePoints(int startX, int startY, int endX, int endY)
{
    std::vector<std::pair<int, int>> outputArray;

    int dx = endX - startX;
    int dy = endY - startY;
    int absdx = std::abs(dx);
    int absdy = std::abs(dy);

    int x = startX;
    int y = startY;
    outputArray.push_back({x, y}); // Add starting point

    // Slope < 1
    if (absdx > absdy)
    {
        int d = 2 * absdy - absdx;

        for (int i = 0; i < absdx; ++i)
        {
            x = dx < 0 ? x - 1 : x + 1;
            if (d < 0)
            {
                d = d + 2 * absdy;
            }
            else
            {
                y = dy < 0 ? y - 1 : y + 1;
                d = d + (2 * absdy - 2 * absdx);
            }
            outputArray.push_back({x, y});
        }
    }
    else
    { // Case when slope is greater than or equals to 1
        int d = 2 * absdx - absdy;

        for (int i = 0; i < absdy; ++i)
        {
            y = dy < 0 ? y - 1 : y + 1;
            if (d < 0)
            {
                d = d + 2 * absdx;
            }
            else
            {
                x = dx < 0 ? x - 1 : x + 1;
                d = d + (2 * absdx - 2 * absdy);
            }
            outputArray.push_back({x, y});
        }
    }
    return outputArray;
}

std::vector<Node> PRM::createNodesAndEdges(std::vector<Node> Graph_)
{
    float max_distance = 1000;
    int edges = 0;
    int barWidth = 70;

    std::cout << "Starting PRM::createNodesAndEdges with " << Graph_.size() << " nodes.\n";

    for (size_t k = 0; k < Graph_.size(); k++)
    {
        std::vector<Node> local_nodes;
        std::vector<std::pair<float, size_t>> distances;

        std::cout << "\nProcessing Node " << k << " with ID " << Graph_[k].id << " at position (" << Graph_[k].x << ", " << Graph_[k].y << ")\n";

        for (size_t m = 0; m < Graph_.size(); m++)
        {
            if (Graph_[m].id != Graph_[k].id)
            { // Skip the target node itself
                float distance = sqrt(pow(Graph_[k].x - Graph_[m].x, 2) + pow(Graph_[k].y - Graph_[m].y, 2));
                distances.push_back({distance, m});
                // std::cout << "  Distance to Node " << m << " (ID " << Graph_[m].id << "): " << distance << "\n";
            }
        }

        size_t count = 30;
        size_t numResults = std::min(count, distances.size());
        std::partial_sort(distances.begin(), distances.begin() + numResults, distances.end());

        std::cout << "  Closest " << numResults << " nodes found.\n";

        // Collect the closest nodes
        std::vector<Node> closestNodes;
        for (size_t i = 0; i < numResults; ++i)
        {
            local_nodes.push_back(Graph_[distances[i].second]);
            std::cout << "    Closest Node " << i << ": ID " << local_nodes.back().id << " at position (" << local_nodes.back().x << ", " << local_nodes.back().y << ")\n";
        }

        for (size_t j = 0; j < local_nodes.size(); j++)
        {
            if (pathIsClear(Graph_[k], local_nodes[j]))
            {
                Graph_[k].edges.push_back(local_nodes[j].id);
                edges++;
                std::cout << "    Edge added between Node " << Graph_[k].id << " and Node " << local_nodes[j].id << "\n";
            }
            else
            {
                std::cout << "    No clear path between Node " << Graph_[k].id << " and Node " << local_nodes[j].id << "\n";
            }
        }

        // Update progress bar
        float progress = static_cast<float>(k + 1) / Graph_.size();
        std::cout << "[";
        int pos = barWidth * progress;
        for (int i = 0; i < barWidth; ++i)
        {
            if (i < pos)
                std::cout << "=";
            else if (i == pos)
                std::cout << ">";
            else
                std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();
    }

    std::cout << std::endl; // Ensure the progress bar moves to the next line after completion
    std::cout << "Total edges created: " << edges << std::endl;

    return Graph_;
}


// Visualisation
///////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat PRM::Load_Map()
{
    // READ Image
    std::cout << "Reading the map image from '/home/liam/map.pgm'...\n";
    cv::Mat grayscaleMapImage = cv::imread("/home/liam/map.pgm", cv::IMREAD_GRAYSCALE);
    if (grayscaleMapImage.empty())
    {
        std::cerr << "Could not open or find the map image" << std::endl;
        return cv::Mat();  // Return an empty Mat if the image could not be loaded
    }

    std::cout << "Successfully loaded the grayscale map image.\n";

    cv::Mat mapImage;
    cv::cvtColor(grayscaleMapImage, mapImage, cv::COLOR_GRAY2BGR);
    std::cout << "Converted grayscale map image to BGR.\n";

    cv::flip(mapImage, mapImage, 0);
    std::cout << "Flipped the map image vertically.\n";

    return mapImage;
}

cv::Mat PRM::visalise_prm(cv::Mat mapImage, std::vector<Node> Graph_)
{
    int radius = 0;              // You can adjust this value as needed
    cv::Scalar color(0, 0, 255); // BGR value for red

    // std::cout << "Starting visualization of PRM on the map image.\n";
    // std::cout << "Map image size: " << mapImage.cols << "x" << mapImage.rows << "\n";
    // std::cout << "Number of nodes in graph: " << Graph_.size() << "\n";

    // Draw edges between nodes
    for (size_t k = 0; k < Graph_.size(); k++)
    {
        cv::Point center(Graph_[k].x, Graph_[k].y);
        // std::cout << "Processing Node " << k << " at (" << center.x << ", " << center.y << ")\n";

        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows)
        {
            // std::cout << "Node " << k << " is within map bounds.\n";
            for (size_t l = 0; l < Graph_.at(k).edges.size(); l++)
            {
                int index = Graph_.at(k).edges.at(l);
                const auto &connected_node = Graph_[index];

                cv::Point connected_node_center(connected_node.x, connected_node.y);
                // std::cout << "  Drawing line from Node " << k << " to connected Node " << index
                        //   << " at (" << connected_node_center.x << ", " << connected_node_center.y << ").\n";

                // Correcting thickness to be at least 1
                int thickness = 1; // Ensure thickness is a valid positive integer
                cv::line(mapImage, center, connected_node_center, cv::Scalar(0, 255, 255), thickness);
            }
        }
        else
        {
            // std::cout << "Node " << k << " is out of map bounds, skipping edge drawing.\n";
        }
    }

    // Draw nodes
    for (size_t k = 0; k < Graph_.size(); k++)
    {
        cv::Point center(Graph_[k].x, Graph_[k].y);
        // std::cout << "Drawing circle for Node " << k << " at (" << center.x << ", " << center.y << ")\n";

        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows)
        {
            cv::circle(mapImage, center, radius, cv::Scalar(255, 0, 0), -1);
            // std::cout << "Circle drawn for Node " << k << ".\n";
        }
        else
        {
            // std::cout << "Node " << k << " is out of map bounds, skipping circle drawing.\n";
        }
    }

    // std::cout << "Completed visualization of PRM on the map image.\n";
    return mapImage;
}
cv::Mat PRM::visalise_PRM_With_Path(std::vector<int> path, cv::Mat mapImage, std::vector<Node> Graph_)
{

    for (int x = 0; x < path.size(); x++)
    {
        // std::cout << "STUCK" << x << "of " << path.size()  << std::endl;ast
        cv::Point prev;
        prev.x = Graph_.at(path.at(x - 1)).x;
        prev.y = Graph_.at(path.at(x - 1)).y;
        cv::Point temp;
        temp.x = Graph_.at(path.at(x)).x;
        temp.y = Graph_.at(path.at(x)).y;
        cv::line(mapImage, temp, prev, cv::Scalar(0, 0, 255) /*blue*/, 2 /*thickness*/);
    }

    for (int x = 0; x < path.size(); x++)
    {
        // std::cout << "STUCK" << x << "of " << path.size()  << std::endl;
        cv::Point temp;
        temp.x = Graph_.at(path.at(x)).x;
        temp.y = Graph_.at(path.at(x)).y;

        cv::circle(mapImage, temp, 0, cv::Scalar(255, 0, 0), -1);
    }

    return mapImage;
}


void PRM::show_map(cv::Mat mapImage)
{
    cv::namedWindow("SLAM Map with Nodes", cv::WINDOW_AUTOSIZE);
    cv::imshow("SLAM Map with Nodes", mapImage);
}

void PRM::save_map(cv::Mat mapImage)
{
    cv::imwrite("/home/liam/Desktop/map_with_nodes fixed!!!.png", mapImage);
}



// Path Planni
///////////////////////////////////////////////////////////////////////////////////////

void PRM::findPath(int startNodeId, int goalNodeId)
{
    // A star
}

std::vector<int> PRM::findPathDijkstra(const std::vector<Node> &graph, int startId, int targetId)
{
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, ComparePair> pq;
    std::unordered_map<int, float> dist; // Distance from start node to each node
    std::unordered_map<int, int> prev;   // Previous node in optimal path from source
    std::vector<int> path;               // Store the final path

    // std::cout << "PRM size " << graph.size() << std::endl;

    // Initialize distances as infinity
    for (const auto &node : graph)
    {
        dist[node.id] = std::numeric_limits<float>::infinity();
    }

    dist[startId] = 0.0; // Distance from start node to itself is zero
    pq.push({0.0, startId});

    while (!pq.empty())
    {
        int currentNodeId = pq.top().second;
        pq.pop();

        // std::cout << "loopping" <<std::endl;
        if (currentNodeId == targetId)
        { // If target node is reached
            // std::cout << "Total distance from start to target: " << dist[targetId] << std::endl; // Print the total distance to target
            while (currentNodeId != startId)
            { // Reconstruct the path
                path.push_back(currentNodeId);
                currentNodeId = prev[currentNodeId];
            }
            path.push_back(startId);                // Add start node at the end
            std::reverse(path.begin(), path.end()); // Reverse to get the correct order from start to target
            return path;
        }

        // Explore the neighbors of the current node
        for (const auto &edgeId : graph[currentNodeId].edges)
        {
            float alt = dist[currentNodeId] + nodeDistance(graph[currentNodeId], graph[edgeId]);
            if (alt < dist[edgeId])
            {
                dist[edgeId] = alt;
                prev[edgeId] = currentNodeId;
                pq.push({alt, edgeId});
            }
        }
    }
    return path; // Return an empty path if no path is found
}

std::vector<int> PRM::findPathAStar(const std::vector<Node> &graph, int startId, int targetId)
{
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, ComparePair> pq;
    std::unordered_map<int, float> dist; // Distance from start node to each node
    std::unordered_map<int, int> prev;   // Previous node in optimal path from source
    std::vector<int> path;               // Store the final path

    // Initialize distances as infinity
    for (const auto &node : graph)
    {
        dist[node.id] = std::numeric_limits<float>::infinity();
    }

    dist[startId] = 0.0;                                                          // Distance from start node to itself is zero
    pq.push({0.0 + euclideanDistance(graph[startId], graph[targetId]), startId}); // Include heuristic value in priority queue

    while (!pq.empty())
    {
        int currentNodeId = pq.top().second;
        pq.pop();

        if (currentNodeId == targetId)
        { // If target node is reached
            // Reconstruct the path
            while (currentNodeId != startId)
            {
                path.push_back(currentNodeId);
                currentNodeId = prev[currentNodeId];
            }
            path.push_back(startId);                // Add start node at the end
            std::reverse(path.begin(), path.end()); // Reverse to get the correct order from start to target
            return path;
        }

        // Explore the neighbors of the current node
        for (const auto &edgeId : graph[currentNodeId].edges)
        {
            float alt = dist[currentNodeId] + euclideanDistance(graph[currentNodeId], graph[edgeId]);
            if (alt < dist[edgeId])
            {
                dist[edgeId] = alt;
                prev[edgeId] = currentNodeId;
                pq.push({alt + euclideanDistance(graph[edgeId], graph[targetId]), edgeId}); // Include heuristic value in priority queue
            }
        }
    }

    return path; // Return an empty path if no path is found
}

std::vector<int> PRM::findPathAStarWithBlackList(const std::vector<Node> &graph, int startId, int targetId, const std::unordered_set<int> &blacklist)
{
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, ComparePair> pq;
    std::unordered_map<int, float> dist; // Distance from start node to each node
    std::unordered_map<int, int> prev;   // Previous node in optimal path from source
    std::vector<int> path;               // Store the final path

    // Initialize distances as infinity
    for (const auto &node : graph)
    {
        dist[node.id] = std::numeric_limits<float>::infinity();
    }

    dist[startId] = 0.0;                                                          // Distance from start node to itself is zero
    pq.push({0.0 + euclideanDistance(graph[startId], graph[targetId]), startId}); // Include heuristic value in priority queue

    while (!pq.empty())
    {
        int currentNodeId = pq.top().second;
        pq.pop();

        // Skip nodes that are in the blacklist
        if (blacklist.find(currentNodeId) != blacklist.end())
        {
            continue;
        }

        if (currentNodeId == targetId)
        { // If target node is reached
            // Reconstruct the path
            while (currentNodeId != startId)
            {
                path.push_back(currentNodeId);
                currentNodeId = prev[currentNodeId];
            }
            path.push_back(startId);                // Add start node at the end
            std::reverse(path.begin(), path.end()); // Reverse to get the correct order from start to target
            return path;
        }

        // Explore the neighbors of the current node
        for (const auto &edgeId : graph[currentNodeId].edges)
        {
            // Skip neighbors that are in the blacklist
            if (blacklist.find(edgeId) != blacklist.end())
            {
                continue;
            }

            float alt = dist[currentNodeId] + euclideanDistance(graph[currentNodeId], graph[edgeId]);
            if (alt < dist[edgeId])
            {
                dist[edgeId] = alt;
                prev[edgeId] = currentNodeId;
                pq.push({alt + euclideanDistance(graph[edgeId], graph[targetId]), edgeId}); // Include heuristic value in priority queue
            }
        }
    }

    return path; // Return an empty path if no path is found
}

std::unordered_set<int> PRM::GenerateBlackList(std::vector<geometry_msgs::msg::Point> CollisonPoints)
{
    std::unordered_set<int> BlackList;
    for (int i = 0; i < CollisonPoints.size(); i++)
    {
        int id = findClosestNode(CollisonPoints.at(i));
        BlackList.insert(id);
        std::vector<int> local_Node_id = getNodesInArea(id, 4);
        for (auto elemet : local_Node_id)
        {
            BlackList.insert(elemet);
        }
    }

    return BlackList;
}

int PRM::findClosestNode(const geometry_msgs::msg::Point &point)
{
    geometry_msgs::msg::Point map_point = convertPointToNodeCordinate(point);
    int closestNodeId = -1;
    float minDistance = std::numeric_limits<float>::infinity();

    for (const auto &node : Graph)
    {
        float dx = node.x - map_point.x;
        float dy = node.y - map_point.y;
        float distance = std::sqrt(dx * dx + dy * dy);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestNodeId = node.id;
        }
    }

    return closestNodeId;
}

std::vector<int> PRM::getNodesInArea(int nodeId, int areaSize)
{
    std::vector<int> nodesInArea;
    auto it = std::find_if(Graph.begin(), Graph.end(), [nodeId](const Node &node)
                           { return node.id == nodeId; });
    if (it == Graph.end())
    {
        return nodesInArea;
    }
    const Node &centerNode = *it;

    for (const auto &node : Graph)
    {
        int dx = std::abs(node.x - centerNode.x);
        int dy = std::abs(node.y - centerNode.y);
        if (dx <= areaSize && dy <= areaSize)
        {
            nodesInArea.push_back(node.id);
        }
    }

    return nodesInArea;
}

// Converting cordinate planes
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float PRM::nodeDistance(const Node &a, const Node &b)
{
    // Simple Euclidean distance for now
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

float PRM::euclideanDistance(const Node &node1, const Node &node2)
{
    return sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2));
}

double PRM::calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

int PRM::setGoalNode(geometry_msgs::msg::Point goal)
{
    // Convert world coordinates to map coordinates
    // std::cout << " Open---------------------------------------" << std::endl;
    geometry_msgs::msg::Point map_point;
    map_point.x = (goal.x - latestMapMetaData_.origin.position.x) / latestMapMetaData_.resolution;
    map_point.y = (goal.y - latestMapMetaData_.origin.position.y) / latestMapMetaData_.resolution;

    // std::cout << "goal x" << goal.x << " y --" << goal.y << std::endl;
    // std::cout << "map x" << map_point.x << " y --" << map_point.y << std::endl;

    int closestNodeId = -1;
    float minDistance = std::numeric_limits<float>::infinity();

    // Iterate through all nodes in the graph to find the closest one
    for (const auto &node : Graph)
    {
        float dx = node.x - map_point.x;
        float dy = node.y - map_point.y;
        float distance = sqrt(dx * dx + dy * dy);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestNodeId = node.id;
        }
    }

    return closestNodeId;
}

geometry_msgs::msg::Point PRM::convertNodeToPoint(Node temp)
{
    geometry_msgs::msg::Point world_point;

    world_point.x = ((temp.x * latestMapMetaData_.resolution) + latestMapMetaData_.origin.position.x);
    world_point.y = ((temp.y * latestMapMetaData_.resolution) + latestMapMetaData_.origin.position.y);

    // std::cout << "point x" << world_point.x << " y --" << world_point.y << std::endl;

    return world_point;
}

geometry_msgs::msg::Point PRM::convertPointToNodeCordinate(geometry_msgs::msg::Point temp)
{
    geometry_msgs::msg::Point temp2;

    // Reverse the transformation from world coordinates to grid coordinates
    temp2.x = (temp.x - latestMapMetaData_.origin.position.x) / latestMapMetaData_.resolution;
    temp2.y = (temp.y - latestMapMetaData_.origin.position.y) / latestMapMetaData_.resolution;

    return temp2;
}

cv::Point PRM::convertPointToNodeCordinate(cv::Point temp)
{
    cv::Point temp2;

    // Reverse the transformation from world coordinates to grid coordinates
    temp2.x = (temp.x - latestMapMetaData_.origin.position.x) / latestMapMetaData_.resolution;
    temp2.y = (temp.y - latestMapMetaData_.origin.position.y) / latestMapMetaData_.resolution;

    return temp2;
}

double PRM::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat)
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w); 
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

std::vector<Node> PRM::rotateNodes(std::vector<Node> &graph, const geometry_msgs::msg::Quaternion &orientation, const geometry_msgs::msg::Pose &mapOrigin)
{
    double yaw = getYawFromQuaternion(orientation);
    float originX = mapOrigin.position.x; // X coordinate of the origin
    float originY = mapOrigin.position.y; // Y coordinate of the origin

    for (Node &node : graph)
    {
        // Translate points to rotate around the origin (0,0)
        float translatedX = node.x - originX;
        float translatedY = node.y - originY;

        // Perform rotation
        float rotatedX = translatedX * cos(yaw) - translatedY * sin(yaw);
        float rotatedY = translatedX * sin(yaw) + translatedY * cos(yaw);

        // Translate points back
        node.x = rotatedX + originX;
        node.y = rotatedY + originY;
    }

    return graph;
}

std::vector<geometry_msgs::msg::Point> PRM::ConvertParthToWorld(std::vector<int> path, std::vector<Node> Graph_)
{
    std::vector<geometry_msgs::msg::Point> path_World;
    Node temp;
    for (int x = 0; x < path.size(); x++)
    {

        for (int y = 0; y < Graph_.size(); y++)
        {

            if (Graph_.at(y).id == path.at(x))
            {

                temp = Graph_.at(y);
            }
        }
        path_World.push_back(convertNodeToPoint(temp));
    }

    return path_World;
}

bool PRM::newPoint(geometry_msgs::msg::Point point, std::vector<Node> temp)
{
    int x1 = static_cast<int>(point.x);
    int y1 = static_cast<int>(point.y);
    for (auto node : temp)
    {
        int x2 = static_cast<int>(node.x);
        int y2 = static_cast<int>(node.y);

        if (x1 == x2 || y1 == y2)
        {
            return false;
        }
    }
    return true;
}

// Open CV USER DEFINED BACKGROUND REMOVAL
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PRM::staticRemoveNodeCallback(int event, int x, int y, int flags, void *userdata)
{
    PRM *self = static_cast<PRM *>(userdata);
    self->removeNodeCallback(event, x, y, flags, userdata);
}

void PRM::removeNodeCallback(int event, int x, int y, int flags, void *)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Adjust the click coordinates to account for the zoom factor
        double zoomFactor = 2.0; // Ensure this matches the zoom factor used in the display function
        int adjustedX = static_cast<int>(x / zoomFactor);
        int adjustedY = static_cast<int>(y / zoomFactor);

        // Find and remove nodes that are within a 3x3 area of the clicked point
        std::vector<size_t> nodesToRemove;

        for (size_t i = 0; i < Graph.size(); ++i)
        {
            double dist = cv::norm(cv::Point(adjustedX, adjustedY) - cv::Point(Graph[i].x, Graph[i].y));
            if (dist <= 3.0)
            { // 3x3 area threshold
                nodesToRemove.push_back(i);
            }
        }

        for (size_t i = 0; i < nodesToRemove.size(); ++i)
        {
            size_t closestNodeIndex = nodesToRemove[i];
            std::cout << "Removing node at (" << Graph[closestNodeIndex].x << ", " << Graph[closestNodeIndex].y << ")" << std::endl;

            // Remove edges pointing to the deleted node
            int deletedNodeId = Graph[closestNodeIndex].id;
            for (auto &node : Graph)
            {
                node.edges.erase(std::remove(node.edges.begin(), node.edges.end(), deletedNodeId), node.edges.end());
            }

            // Set the node coordinates out of bounds to effectively remove it
            Graph.at(closestNodeIndex).edges.clear();
            Graph.at(closestNodeIndex).x = 99999999;
            Graph.at(closestNodeIndex).y = 99999999;
        }
    }
}

cv::Mat PRM::removeNodes(cv::Mat mapImage, std::vector<Node> &Graph_)
{
    std::cout << "Starting PRM::removeNodes function.\n";

    // Set up the window and callback
    cv::namedWindow("Remove Nodes");
    cv::resizeWindow("Remove Nodes", 1200, 800);
    std::cout << "Window 'Remove Nodes' created and resized to 1200x800.\n";

    // Set mouse callback to allow for node removal
    cv::setMouseCallback("Remove Nodes", PRM::staticRemoveNodeCallback, this);
    std::cout << "Mouse callback set for window 'Remove Nodes'.\n";

    cv::Mat clone = mapImage.clone();

    while (true)
    {
        // std::cout << "Entering the display loop.\n";

        cv::Mat tempImage = visalise_prm(clone, Graph_);
        // std::cout << "PRM visualized on temporary image.\n";

        cv::Mat zoomedImage;
        double zoomFactor = 2.0; // Adjust this factor to zoom in/out
        cv::resize(tempImage, zoomedImage, cv::Size(), zoomFactor, zoomFactor, cv::INTER_LINEAR);
        // std::cout << "Image resized for zoom with factor " << zoomFactor << ".\n";

        cv::imshow("Remove Nodes", zoomedImage);
        // std::cout << "Displaying zoomed image in 'Remove Nodes' window.\n";

        int key = cv::waitKey(1);
        // std::cout << "Key pressed: " << key << "\n";

        if (key == 27)
        { // ESC key to exit
            std::cout << "ESC key pressed. Exiting display loop.\n";
            break;
        }
    }

    cv::destroyWindow("Remove Nodes");
    std::cout << "Window 'Remove Nodes' destroyed.\n";

    Graph_ = Graph_; // Update the original graph with the modified graph
    std::cout << "Graph updated with modified nodes.\n";

    std::cout << "Exiting PRM::removeNodes function.\n";
    return mapImage;
}

// OPEN CV USER DEFINED BOUNDARIES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PRM::staticMouseCallback(int event, int x, int y, int flags, void *userdata)
{
    PRM *self = static_cast<PRM *>(userdata);
    // std::cout << "staticMouseCallback called" << std::endl;
    self->mouseCallback(event, x, y, flags, userdata);
}

void PRM::mouseCallback(int event, int x, int y, int flags, void *)
{
    // std::cout << "mouseCallback called with event: " << event << ", x: " << x << ", y: " << y << std::endl;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        polygonPoints.push_back(cv::Point(x, y));
        // std::cout << "Point added: (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        if (!polygonPoints.empty())
        {
            polygonPoints.pop_back();
            // std::cout << "Last point removed" << std::endl;
        }
    }
}

std::vector<cv::Point> PRM::getUserDefinedPolygon(const std::string &mapImagePath)
{
    cv::Mat image = cv::imread(mapImagePath, cv::IMREAD_GRAYSCALE); // Load as grayscale
    if (image.empty())
    {
        std::cerr << "Error: Could not load map image." << std::endl;
        return {};
    }
    // std::cout << "Map image loaded successfully" << std::endl;

    // Convert grayscale to BGR for drawing
    cv::Mat colorImage;
    cv::cvtColor(image, colorImage, cv::COLOR_GRAY2BGR);
    // cv::flip(colorImage, colorImage, 0);
    // std::cout << "image processed" << std::endl;

    // Set up the window and callback
    cv::namedWindow("Draw Polygon");
    // std::cout << "window created" << std::endl;
    cv::setMouseCallback("Draw Polygon", PRM::staticMouseCallback, this);
    // std::cout << "callback created" << std::endl;

    while (true)
    {
        cv::Mat tempImage = image.clone();
        if (polygonPoints.size() > 1)
        {
            for (size_t i = 0; i < polygonPoints.size() - 1; ++i)
            {
                cv::line(tempImage, polygonPoints[i], polygonPoints[i + 1], cv::Scalar(0, 0, 0), 2);
            }
            cv::line(tempImage, polygonPoints.back(), polygonPoints.front(), cv::Scalar(0, 0, 0), 2);
        }

        cv::imshow("Draw Polygon", tempImage);
        int key = cv::waitKey(1);
        if (key == 27)
        { // ESC key to exit
            std::cout << "ESC key pressed, exiting polygon drawing" << std::endl;
            break;
        }
    }
    std::cout << "about to destroy window" << std::endl;
    cv::waitKey(1);
    cv::destroyAllWindows();

    // cv::destroyWindow("Draw Polygon");
    std::cout << "Polygon drawing window destroyed" << std::endl;
    // Flip the polygon points on both x and y axes
    // std::vector<cv::Point> flippedPolygonPoints;
  std::vector<cv::Point> flippedPolygonPoints;
    cv::Point center(colorImage.cols / 2, colorImage.rows / 2);

    std::cout << "Image Center: (" << center.x << ", " << center.y << ")" << std::endl;
    std::cout << "Original Polygon Points:" << std::endl;

    // Print original points
    for (const auto& point : polygonPoints) {
        std::cout << "Point: (" << point.x << ", " << point.y << ")" << std::endl;
    }

    // Flip each point
    for (const auto& point : polygonPoints) {
        int flippedY = center.y + (center.y - point.y);
        int flippedX = center.x - (center.x - point.x);

        // Debug output for each flipped point calculation
        std::cout << "Original Point: (" << point.x << ", " << point.y << ")"
                  << " -> Flipped Point: (" << flippedX << ", " << flippedY << ")" << std::endl;

        flippedPolygonPoints.push_back(cv::Point(flippedX, flippedY));
    }

    std::cout << "Flipped Polygon Points:" << std::endl;

    // Print flipped points
    for (const auto& point : flippedPolygonPoints) {
        std::cout << "Flipped Point: (" << point.x << ", " << point.y << ")" << std::endl;
    }

    return flippedPolygonPoints;
}



bool PRM::isPointInPolygon(const cv::Point &point, const std::vector<cv::Point> &polygon)
{
    bool result = cv::pointPolygonTest(polygon, point, false) >= 0;
    // std::cout << "isPointInPolygon called with point: (" << point.x << ", " << point.y << "), result: " << result << std::endl;
    return result;
}

std::vector<Node> PRM::samplePointsCV()
{
    // std::cout << "samplePointsCV called" << std::endl;

    // Obtain user-defined polygon
    std::vector<cv::Point> inclusionPolygon = getUserDefinedPolygon("/home/liam/map.pgm");
    // std::cout << "User-defined polygon obtained" << std::endl;
    Graph.clear();
    int id = 0;
    std::cout << latestMapMetaData_.width << std::endl;
    std::cout << latestMapMetaData_.height << std::endl;
    


    // Random generators
    std::random_device rd;                                                 // Obtain a random number from hardware
    std::mt19937 eng(rd());                                                // Seed the generator
    std::uniform_real_distribution<> distrX(0, latestMapMetaData_.width);   // Define the range for x
    std::uniform_real_distribution<> distrY(0, latestMapMetaData_.height);   // Define the range for y

    double Max_y = 0;
    double min_y = 9999999;

    std::cout << "Starting graph generation with number of points: " << numberOfPoints_ << std::endl;

    while (Graph.size() < numberOfPoints_) {
        geometry_msgs::msg::Point point;
        point.x = distrX(eng);
        point.y = distrY(eng);

        std::cout << "Random point generated: (" << point.x << ", " << point.y << ")" << std::endl;

        cv::Point cvPoint(point.x, point.y);

        if (isPointInPolygon(cvPoint, inclusionPolygon) && ValidPoint(point)) {
            Node temp;
            temp.x = point.x;
            temp.y = point.y;
            temp.id = id;
            id++;
            Graph.push_back(temp);

            // std::cout << "Point added to graph: (" << temp.x << ", " << temp.y << "), ID: " << temp.id << std::endl;

            if (temp.y > Max_y) {
                Max_y = temp.y;
                // std::cout << "New Max Y updated: " << Max_y << std::endl;
            }
            if (temp.y < min_y) {
                min_y = temp.y;
                // std::cout << "New Min Y updated: " << min_y << std::endl;
            }
        } else {
            std::cout << "Point not added: (" << point.x << ", " << point.y << ") - Not in polygon or invalid" << std::endl;
        }
    }

    std::cout << "Graph generation complete. Total points added: " << Graph.size() << std::endl;
    std::cout << "Final Max Y: " << Max_y << ", Min Y: " << min_y << std::endl;

    return Graph;
}