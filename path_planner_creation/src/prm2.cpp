// Graph.cpp

#include "prm2.hpp"
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <vector>
#include <utility>  // For std::pair
#include <fstream>

Graph::Graph()
{
    Id = 0;
}

void Graph::UpdateMapData(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData MapMetaData_)
{
    std::cout << "PRM map data openning" << std::endl;
    SlamMapData = map;
    numberOfPoints_ = 10000;
    latestMapMetaData_ = MapMetaData_;
}




// A* heuristic (Euclidean distance)
double Graph::heuristic(const Node& a, const Node& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}


std::vector<int> Graph::a_star(const std::unordered_map<int, Node>& nodes, int start_id, int finish_id) {
    // Priority queue for exploring nodes, ordered by f_cost
    using QueueElement = std::pair<double, int>; // (f_cost, node_id)
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> open_set;

    // Maps to store g_cost and parent node
    std::unordered_map<int, double> g_cost; // Cost from start node to current node
    std::unordered_map<int, int> came_from; // Parent node

    // Ensure the start and finish nodes are valid
    if (nodes.find(start_id) == nodes.end() || nodes.find(finish_id) == nodes.end()) {
        std::cerr << "Error: Invalid start or finish node ID." << std::endl;
        return {};
    }

    // Initialize g_cost with infinity
    for (const auto& pair : nodes) {
        g_cost[pair.first] = std::numeric_limits<double>::infinity();
    }

    // Start node's g_cost is 0
    g_cost[start_id] = 0.0;

    // Insert start node into the priority queue with f_cost = heuristic
    open_set.emplace(heuristic(nodes.at(start_id), nodes.at(finish_id)), start_id);
    std::cout << "Starting A* from node " << start_id << " to node " << finish_id << std::endl;

    while (!open_set.empty()) {
        // Get the node with the lowest f_cost
        int current_id = open_set.top().second;
        open_set.pop();
        std::cout << "Processing node: " << current_id << std::endl;

        // If we've reached the goal, reconstruct the path
        if (current_id == finish_id) {
            std::cout << "Goal node " << finish_id << " reached." << std::endl;
            std::vector<int> path;
            while (current_id != start_id) {
                path.push_back(current_id);
                current_id = came_from[current_id];
            }
            path.push_back(start_id);
            std::reverse(path.begin(), path.end());
            std::cout << "Path found: ";
            for (int id : path) {
                std::cout << id << " ";
            }
            std::cout << std::endl;
            return path; // Return the path from start to finish
        }

        // Explore neighbors
        const Node& current_node = nodes.at(current_id);
        for (const auto& [neighbor_id, cost] : current_node.edges) {
            double tentative_g_cost = g_cost[current_id] + cost;
            std::cout << "Checking neighbor " << neighbor_id << " with cost " << cost << std::endl;

            // If this path to neighbor is better, update
            if (tentative_g_cost < g_cost[neighbor_id]) {
                std::cout << "Updating g_cost and came_from for neighbor " << neighbor_id << std::endl;
                came_from[neighbor_id] = current_id;
                g_cost[neighbor_id] = tentative_g_cost;
                double f_cost = tentative_g_cost + heuristic(nodes.at(neighbor_id), nodes.at(finish_id));
                open_set.emplace(f_cost, neighbor_id);
            }
        }
    }

    // If we exit the loop, there is no path
    std::cerr << "No path found!" << std::endl;
    return {};
}



void Graph::Generate_Map()
{

    
    std::vector<Polygons> userPolygon = getUserDefinedPolygons("/home/liam/map.pgm");
    Lane_polygons = userPolygon;
    
    generateGridNodes(SlamMapData, userPolygon);
    std::cout << "Grid done" << std::endl;
    generate_Prm_Nodes();
    std::cout << "Prm done" << std::endl;

    // for (int x = 0; x < nodes.size(); x++) {
    //     std::cout << "Node " << x << ": (" << nodes[x].x << ", " << nodes[x].y << ")" << std::endl;
    // }

//    for (int j = 0; j < SlamMapData.data.size(); j++) {
//         std::cout << "grid[" << j << "] = " << static_cast<int>(SlamMapData.data.at(j)) << std::endl;
//     }

    std::vector<int> path = a_star(nodes, 5, 999);
    std::cout << "A-Star Done" << std::endl;
    std::cout << path.size() << std::endl;


   

    save_nodes(nodes, "/home/liam/git/RS1_Smart_Factory/path_planner_creation/road_map/file.json");
    nodes.clear();
    nodes = load_nodes("/home/liam/git/RS1_Smart_Factory/path_planner_creation/road_map/file.json");


    show_Prm(path);
    // // cv::Mat map = Load_Map();
    // User_path_drawing(map);

    // export

    // load

    //


}


void Graph::Click_Draw_path(){

}



void Graph::show_Prm(std::vector<int> path)
{
    std::cout << "Show PRM Opens" << std::endl;
    cv::Mat MapImage = Load_Map();
    MapImage = visalise_prm(MapImage, nodes);
    MapImage = visalise_path(MapImage, path);
    save_map(MapImage);
    show_map(MapImage);
}

cv::Mat Graph::visalise_path(cv::Mat mapImage, std::vector<int> path) {
    cv::Scalar Line_colour(0, 0, 255); // BGR value for red
    if (path.size() == 0){
        return mapImage;
    }

    for (int i = 0; i < path.size()-1; i++){
        cv::Point start(nodes.at(path.at(i)).x, nodes.at(path.at(i)).y);

        cv::Point end(nodes.at(path.at(i+1)).x, nodes.at(path.at(i+1)).y);

        cv::line(mapImage, start, end, Line_colour, 1); // Draw the edge
    }
    return mapImage;
}


void Graph::show_map(cv::Mat mapImage)
{
    cv::namedWindow("SLAM Map with Nodes", cv::WINDOW_AUTOSIZE);
    cv::imshow("SLAM Map with Nodes", mapImage);
}

void Graph::save_map(cv::Mat mapImage)
{
    cv::imwrite("/home/liam/Desktop/fixed_warhouse_map_with_nodes!!!.png", mapImage);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat Graph::Load_Map()
{
    // READ Image
    std::cout << "Reading the map image from '/home/liam/map.pgm'...\n";
    cv::Mat grayscaleMapImage = cv::imread("/home/liam/map.pgm", cv::IMREAD_GRAYSCALE);
    if (grayscaleMapImage.empty())
    {
        std::cerr << "Could not open or find the map image" << std::endl;
        return cv::Mat(); // Return an empty Mat if the image could not be loaded
    }

    std::cout << "Successfully loaded the grayscale map image.\n";

    cv::Mat mapImage;
    cv::cvtColor(grayscaleMapImage, mapImage, cv::COLOR_GRAY2BGR);
    std::cout << "Converted grayscale map image to BGR.\n";

    // cv::flip(mapImage, mapImage, 0);
    cv::flip(mapImage, mapImage, 0); // Flip around the x-axis
    std::cout << "Flipped the map image vertically.\n";

    return mapImage;
}

cv::Mat Graph::visalise_prm(cv::Mat mapImage, std::unordered_map<int, Node> graph_struct)
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

void Graph::generateGridNodes(const nav_msgs::msg::OccupancyGrid msg, const std::vector<Polygons> polygons)
{
    int width = msg.info.width;
    int height = msg.info.height;
    double resolution = msg.info.resolution;
    auto origin = msg.info.origin.position;

    
    int id_start;

    for (int c = 0; c < polygons.size(); c++){
        id_start = Id;
        for (int y = 0; y < height; y =y+2) {
            for (int x = 0; x < width; x= x+2){ // could optimse this by getting the maxium with of polygon rather than map width. 

                int index = x + y * width;
                int map_index= static_cast<int>(SlamMapData.data.at(index));
                
                geometry_msgs::msg::Point map_check;
                map_check.x = x;
                map_check.y = y;

                if (isInsidePolygon(polygons.at(c).Polyon_Points, x, y)) { // Check if point is inside user-defined polygon
                    if (validate_point(map_check)) { // Free space // obstacles but not unknown spaces. 
                        double world_x = origin.x + x * resolution;
                        double world_y = origin.y + y * resolution; 
                        // std::cout << map_index << std::endl;    
                        addNode(Id, x, y);
                        std::cout << "adding_node" << std::endl;
                        Id++;
                    }
                }   
            }
        }
        std::cout << id_start << std::endl;
        std::cout << Id << std::endl;
        int id_start_loop = id_start;
        for (int j = id_start; j < Id; j++){
            std::cout << "Processing node with id: " << j << std::endl;
            
            int direction = polygons.at(c).direction;
            std::cout << "Direction for polygon: " << direction << std::endl;

            double direction_rads;
            if (direction == 1){
                direction_rads = M_PI;
                std::cout << "Direction in radians: " << direction_rads << " (180 degrees)" << std::endl;
            }
            else if (direction == 2){
                direction_rads = (M_PI)/2;
                std::cout << "Direction in radians: " << direction_rads << " (90 degrees)" << std::endl;
            }
            else if (direction == 3){
                direction_rads = 0;
                std::cout << "Direction in radians: " << direction_rads << " (0 degrees)" << std::endl;
            }
            else if (direction == 4){
                direction_rads = -M_PI;
                std::cout << "Direction in radians: " << direction_rads << " (-180 degrees)" << std::endl;
            }

            
            int y_start = nodes.at(j).y - 5;
            int y_max = y_start + 5;

            std::cout << "Node bounds: x_start = " << nodes.at(j).x -5 << ", x_max = " << nodes.at(j).x + 5 << ", y_start = " << y_start << ", y_max = " << y_max << std::endl;

            for (; y_start < y_max; y_start++){
                int x_start = nodes.at(j).x - 5;
                int x_max = x_start + 10;
                std::cout << "Checking y position: " << y_start << std::endl;
                for (; x_start < x_max; x_start++){
                    std::cout << "Checking x position: " << x_start << std::endl;

                    int found_id = find_node_at_cordinates(x_start, y_start);
                    std::cout << "Found node id at coordinates (" << x_start << ", " << y_start << "): " << found_id << std::endl;

                    if (found_id != -1){
                        std::cout << "Node found! Adding edge..." << std::endl;

                        double node_distance = distance_between_nodes(nodes.at(j).x, nodes.at(j).y, x_start, y_start);
                        std::cout << "Node distance: " << node_distance << std::endl;

                        double node_angle = angle_between_nodes(nodes.at(j).x, nodes.at(j).y, x_start, y_start);
                        std::cout << "Node angle: " << node_angle << std::endl;

                        if (node_angle < normalise_angle(direction_rads + M_PI/4) && node_angle > normalise_angle(direction_rads - M_PI/4)){
                            std::cout << "Direction within acceptable range, adding edge with normal cost." << std::endl;
                            addEdge(j, found_id, node_distance);
                        }
                        else if (node_angle < normalise_angle(direction_rads + M_PI/4) && node_angle > normalise_angle(direction_rads - M_PI/4)){
                            std::cout << "Direction less desired, adding edge with increased cost (1.5x)." << std::endl;
                            addEdge(j, found_id, node_distance * 1.5);
                        }
                        else {
                            std::cout << "Undesired direction, adding edge with highest cost (2x)." << std::endl;
                            addEdge(j, found_id, node_distance * 2);
                        }
                    }
                }
            }
        }

    }
}




void Graph::generate_Prm_Nodes(){
    // graph definition
    // Random generators
    std::random_device rd;                                                 // Obtain a random number from hardware
    std::mt19937 eng(rd());                                                // Seed the generator
    std::uniform_real_distribution<> distrX(0, latestMapMetaData_.width);  // Define the range for x
    std::uniform_real_distribution<> distrY(0, latestMapMetaData_.height); // Define the range for y
    

    double boundary_area = calculate_polygon_area(boundary_Polygon);
    double Grid_area;
    for (int i = 0; i < Lane_polygons.size(); i++){
        Grid_area = Grid_area + calculate_polygon_area(Lane_polygons.at(i).Polyon_Points);
    }


    double Prm_area = boundary_area - Grid_area;
    double area_per_node = 1;
    int density = static_cast<int>((Prm_area * 0.4) / area_per_node);;
    
    
     // add cacluation for ratio of free space of
    int max_graph_size = nodes.size() + density;
    int barWidth = 70;
    int start = nodes.size();
    
    while (nodes.size() < max_graph_size)
    {
        geometry_msgs::msg::Point point;
        point.x = distrX(eng);
        point.y = distrY(eng);
        cv::Point cvPoint(point.x, point.y);

        if (Is_Point_In_boundary(cvPoint))
        {
            if (validate_point(point)) {  
                addNode(Id, point.x, point.y);
                Id++;
            }
            else {
                // std::cout << "Point not valid" << std::endl;
            }
        }
        else {
            // std::cout << "Point not in L-shaped area" << std::endl;
        }

        
        // Display progress bar
        // float progress = (float)Graph.size() / numberOfPoints_;
        // std::cout << "[";
        // int pos = barWidth * progress;
        // for (int i = 0; i < barWidth; ++i)
        // {
        //     if (i < pos)
        //         std::cout << "=";
        //     else if (i == pos)
        //         std::cout << ">";
        //     else
        //         std::cout << " ";
        // }
        // std::cout << "] " << int(progress * 100.0) << " %\r";
        // std::cout.flush();
    }   

    
    // add edges herepush_back
    createNodesAndEdges(start, 0);


}

void Graph::createNodesAndEdges(int start_ID, int end_ID) {
    float max_distance = 1000;
    int edges = 0;
    // int barWidth = 70;


    for (size_t k = start_ID; k < nodes.size(); k++)
    {
        std::vector<Node> local_nodes;
        std::vector<std::pair<float, size_t>> distances;

        // std::cout << "\nProcessing Node " << k << " with ID " << nodes[k].id << " at position (" << nodes[k].x << ", " << nodes[k].y << ")\n";

        for (size_t m = 0; m < nodes.size(); m++)
        {
            if (nodes[m].id != nodes[k].id)
            { // Skip the target node itself
                float distance = sqrt(pow(nodes[k].x - nodes[m].x, 2) + pow(nodes[k].y - nodes[m].y, 2));
                distances.push_back({distance, m});
                // std::cout << "  Distance to Node " << m << " (ID " << Graph_[m].id << "): " << distance << "\n";
            }
        }

        size_t count = 20;
        size_t numResults = std::min(count, distances.size());
        std::partial_sort(distances.begin(), distances.begin() + numResults, distances.end());

        // std::cout << "  Closest " << numResults << " nodes found.\n";

        // Collect the closest nodes
        std::vector<Node> closestNodes;
        for (size_t i = 0; i < numResults; ++i) {
            local_nodes.push_back(nodes[distances[i].second]);
            // std::cout << "    Closest Node " << i << ": ID " << local_nodes.back().id << " at position (" << local_nodes.back().x << ", " << local_nodes.back().y << ")\n";
        }

        for (size_t j = 0; j < local_nodes.size(); j++)
        {
            if (pathIsClear(nodes[k], local_nodes[j]))
            {
                // double distance_cost = sqrt(pow(nodes[k].x - nodes[m].x, 2) + pow(nodes[k].y - nodes[m].y, 2));
                addEdge(k, local_nodes.at(j).id, 5);
                addEdge(local_nodes.at(j).id, k, 5);
                
                // nodes[k].edges.push_back(local_nodes[j].id);
                edges++;
                // std::cout << "    Edge added between Node " << nodes[k].id << " and Node " << local_nodes[j].id << "\n";
            }
            else
            {
                // std::cout << "    No clear path between Node " << nodes[k].id << " and Node " << local_nodes[j].id << "\n";
            }
        }

        // Update progress bar
        // float progress = static_cast<float>(k + 1) / nodes.size();
        // std::cout << "[";
        // int pos = barWidth * progress;
        // for (int i = 0; i < barWidth; ++i)
        // {
        //     if (i < pos)
        //         std::cout << "=";
        //     else if (i == pos)
        //         std::cout << ">";
        //     else
        //         std::cout << " ";
        // }
        // std::cout << "] " << int(progress * 100.0) << " %\r";
        // std::cout.flush();
    }

    std::cout << std::endl; // Ensure the progress bar moves to the next line after completion
    // std::cout << "Total edges created: " << edges << std::endl;

}




bool Graph::pathIsClear(Node Node_A, Node Node_B)
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

        // cv::Point pfoint(pair.first, pair.second);
        // path_points_withoutValidation.push_back(point);
    }

    for (const auto &pair : BressenhamPoints)
    {
        geometry_msgs::msg::Point temp;
        temp.x = pair.first;
        temp.y = pair.second;
        if (validate_point(temp))
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






std::vector<std::pair<int, int>> Graph::bresenhamLinePoints(int startX, int startY, int endX, int endY) {
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
            if (d < 0) {
                d = d + 2 * absdx;
            }
            else {
                x = dx < 0 ? x - 1 : x + 1;
                d = d + (2 * absdx - 2 * absdy);
            }
            outputArray.push_back({x, y});
        }
    }
    return outputArray;
}









bool Graph::Is_Point_In_boundary(cv::Point point) {
    // Check if the point is inside or on the boundary of the main boundary polygon
    if (cv::pointPolygonTest(boundary_Polygon, point, false) < 0) {
        return false; // Point is outside the boundary polygon
    }

    // Check if the point is inside any of the lane polygons
    for (const auto& poly : Lane_polygons) {
        if (cv::pointPolygonTest(poly.Polyon_Points, point, false) >= 0) {
            return false; // Point is inside a lane polygon
        }
    }

    return true; // Point is inside the boundary and not inside any lane polygons
}

bool Graph::Is_Point_In_FreeSpace(geometry_msgs::msg::Point point){
    return true;
}



double Graph::distance_between_nodes(int x1, int y1, int x2, int y2){
    double distance = (sqrt(pow(x2-x1,2) + pow(y2-y1,2)));
    return distance;
}


double Graph::angle_between_nodes(int x1, int y1, int x2, int y2){
    double angle = atan2(y2 - y1, x2 - x1);
    return angle;
}

double Graph::normalise_angle(double angle){
    double modified_angle;
    if (angle > M_PI){
        modified_angle - 2*M_PI;
    }
    if (angle < -M_PI ){
        modified_angle + 2*M_PI;
    }
    return modified_angle;
}

int Graph::find_node_at_cordinates(int x, int y) {
    for (int z = 0; z < nodes.size(); z++){
        if (nodes.at(z).x == x && nodes.at(z).y == y){
            return nodes.at(z).id;
        }
    }
    return -1;
}

int Graph::find_closest_node_at_cordinates(int x, int y) {
    double mininal_distance = 9999999;
    int closest_node;
    for (int z = 0; z < nodes.size(); z++){
        double distance = distance_between_nodes(x, y, nodes.at(z).x, nodes.at(z).y);
        if (nodes.at(z).x == x && nodes.at(z).y == y){
            return nodes.at(z).id;
        }
        else if (mininal_distance > distance){
            mininal_distance = distance;
            closest_node = nodes.at(z).id;
        }
    }
    return closest_node;
}




bool Graph::validate_point(geometry_msgs::msg::Point point){

    // Convert the point's x and y to grid indices
    int grid_x = static_cast<int>(point.x);
    int grid_y = static_cast<int>(point.y);

    // Calculate the linear index of the point on the occupancy grid
    int index = grid_x + (grid_y * SlamMapData.info.width);
    
    // Define the size of the grid to check around the point
    int grid_size = 12; 

    // Loop through the grid_size x grid_size area around the point
    for (int dx = -grid_size / 2; dx <= grid_size / 2; ++dx) {
        for (int dy = -grid_size / 2; dy <= grid_size / 2; ++dy) {
            
            // Calculate new point coordinates in grid
            int new_x = grid_x + dx;
            int new_y = grid_y + dy;

            // Check if the new point is within the map boundaries
            if (new_x < 0 || new_x >= SlamMapData.info.width || new_y < 0 || new_y >= SlamMapData.info.height) {
                return false; // If the point is outside the map boundaries, return false
            }

            // Calculate the index of the new point in the occupancy grid
            int new_index = new_x + (new_y * SlamMapData.info.width);

            // Double-check if the index is within valid range
            if (new_index >= SlamMapData.data.size() || new_index < 0) {
                return false;  // This shouldn't happen given the previous checks, but is a safeguard
            }

            // Check if the point is free of obstacles (value should be 0)
            if (SlamMapData.data.at(new_index) != 0) {
                return false;  // If any cell in the grid area is occupied, return false
            }
        }
    }

    return true; // All points in the grid area are free of obstacles
}


void Graph::prm_method(){
    //for (x number of nodes)
        //generate random points
        //check if random point is within main boundary
        //check if random points is outside polygons.
        //Add node
    //

    // Generate nodes edges for PRM nodes.

}

bool Graph::isInsidePolygon(const std::vector<cv::Point> &polygon, double x, double y)
{
    return cv::pointPolygonTest(polygon, cv::Point(x, y), false) >= 0;
}

std::vector<Polygons> Graph::getUserDefinedPolygons(const std::string &mapImagePath)
{
    std::vector<Polygons> Lanes;
    cv::Mat image = cv::imread(mapImagePath, cv::IMREAD_GRAYSCALE); // Load as grayscale
    if (image.empty())
    {
        std::cerr << "Error: Could not load map image." << std::endl;
        Lanes.clear();
        return Lanes;
    }

    // Convert grayscale to BGR for drawing
    cv::Mat colorImage;
    cv::cvtColor(image, colorImage, cv::COLOR_GRAY2BGR);
    cv::flip(colorImage, colorImage, 0); // Flip around the x-axis
    


    // Set up the window and callback
    cv::namedWindow("Draw Polygons");
    cv::setMouseCallback("Draw Polygons", Graph::staticMouseCallback, this);

    while (true)
    {
        cv::Mat tempImage = colorImage.clone();

        if (currentPolygon.size() > 1)
        {
            for (size_t i = 0; i < currentPolygon.size() - 1; ++i)
            {
                std::cout << currentPolygon[i].x << std::endl;
                cv::line(tempImage, currentPolygon[i], currentPolygon[i + 1], cv::Scalar(0, 0, 0), 2);
            }
            cv::line(tempImage, currentPolygon.back(), currentPolygon.front(), cv::Scalar(0, 0, 0), 2);
        }

        cv::imshow("Draw Polygons", tempImage);
        int key = cv::waitKey(1);

        if (key == 27)
        { // ESC key to exit
            break;
        }
        else if (key == 'n')
        { // 'n' key to start a new polygon
            int dir = 0;  
            if (!currentPolygon.empty())
            {
                // lanes.push_back(currentPolygon);
                int direction;
                std::cout << "Enter direction for the current polygon (e.g., east-only, north-south): " << std::endl;
                bool key_pressed = false;
                while (!key_pressed){
                    int key2;
                    key2 = cv::waitKey(1);
                    switch (key2) {
                    case 'w':
                        std::cout << "You pressed 'w' - Moving Up" << std::endl;
                        key_pressed = true;
                        dir = 1;
                        break;
                    case 's':
                        std::cout << "You pressed 's' - Moving Down" << std::endl;
                        key_pressed = true;
                        dir = 2;
                        break;
                    case 'a':
                        key_pressed = true;
                        dir = 3;
                        std::cout << "You pressed 'a' - Moving Left" << std::endl;
                        break;
                    case 'd':
                        key_pressed = true;
                        dir = 4;
                        std::cout << "You pressed 'd' - Moving Right" << std::endl;
                        break;
                    case 'x':
                        key_pressed = true;
                        dir = 10;
                        std::cout << "You pressed 'b' - boundary" << std::endl;
                        break;
                    default:
                        // key_pressed = true;
                        // std::cout << "Invalid key pressed." << std::endl;
                        break;
                    }

                }
                // directions.push_back(direction);
                if (dir == 10){
                    boundary_Polygon = currentPolygon;

                }
                else {
                    Lanes.emplace_back(currentPolygon, 1);
                }
                currentPolygon.clear(); // Clear current polygon to start a new one

            }
        }
    }
    cv::destroyAllWindows();
    return Lanes;
}

void Graph::staticMouseCallback(int event, int x, int y, int flags, void *userdata)
{
    Graph *graph = reinterpret_cast<Graph *>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        graph->currentPolygon.emplace_back(x, y);
        // std::cout << "Point added: (" << x << ", " << y << ")" << std::endl;
    }
}


void Graph::User_path_drawing(const cv::Mat mapImage) {
    
    cv::Mat MapImage = Load_Map();
    MapImage = visalise_prm(MapImage, nodes);
    cv::Mat node_image = MapImage;

    // MapImage = visalise_path(MapImage, path);
    // save_map(MapImage);
    // show_map(MapImage);

    // Set up the window and callback
    cv::namedWindow("Draw path");
    cv::setMouseCallback("Draw path", Graph::staticMouseCallback, this);

    while (true) {
        cv::Mat tempImage = MapImage.clone();

        cv::imshow("Draw path", tempImage);
        int key = cv::waitKey(1);

        if (key == 27)
        { // ESC key to exit
            break;
        }
        else if (currentPolygon.size() == 2)
        { // 'n' key to start a new polygon
            int start_id = find_closest_node_at_cordinates(currentPolygon.at(0).x, currentPolygon.at(0).y);
            int end_id = find_closest_node_at_cordinates(currentPolygon.at(1).x, currentPolygon.at(1).y);
            std::vector<int> path = a_star(nodes,start_id, end_id);
            MapImage = visalise_path(node_image, path);           
        }
    }
    cv::destroyAllWindows();
    // return Lanes;
}







cv::Mat Graph::drawPolygon(cv::Mat image)
{
    if (polygonPoints.size() > 1)
    {
        for (size_t i = 0; i < polygonPoints.size() - 1; ++i)
        {
            cv::line(image, polygonPoints[i], polygonPoints[i + 1], cv::Scalar(0, 0, 0), 2);
        }
        cv::line(image, polygonPoints.back(), polygonPoints.front(), cv::Scalar(0, 0, 0), 2);
    }
    return image;

}

void Graph::addNode(int id, double x, double y)
{
    
    if (nodes.find(id) == nodes.end()) {
        nodes.emplace(id, Node(id, x, y));
    }
    else {
        std::cerr << "Node with id " << id << " already exists. Skipping addition." << std::endl;
    }
}

void Graph::addEdge(int fromId, int toId, double cost){
    std::cout << "adding edge" << std::endl;

    // Check if the 'fromId' node exists, if not, handle it (either insert or throw an error)
    if (nodes.find(fromId) == nodes.end()) {
        std::cerr << "Node with id " << fromId << " does not exist. Cannot add edge." << std::endl;
        return; // Or throw an exception if needed
    }

    // Check if the 'toId' node exists, if not, handle it (either insert or throw an error)
    if (nodes.find(toId) == nodes.end()) {
        std::cerr << "Node with id " << toId << " does not exist. Cannot add edge." << std::endl;
        return; // Or throw an exception if needed
    }

    std::cout << "nodes number of edges before adding" << std::endl;
    std::cout << nodes.at(fromId).edges.size() << std::endl;
    

    // Add the edge (toId and cost) to the 'fromId' node's edges
    nodes[fromId].edges.push_back(std::make_pair(toId, cost));

    std::cout << "nodes number of edges after adding" << std::endl;
    std::cout << nodes.at(fromId).edges.size() << std::endl;
    std::cout << "checking edge added" << std::endl;


    for (int k =0; k < nodes[fromId].edges.size();k++){
        std::cout << nodes.at(fromId).edges.at(k).first << std::endl;
    }

}


double Graph::calculate_polygon_area(const std::vector<cv::Point>& points) {
    int n = points.size();
    if (n < 3) {
        std::cerr << "A polygon must have at least 3 points." << std::endl;
        return 0.0;
    }

    double area = 0.0;

    // Apply Shoelace Theorem
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n; // Next vertex (wraps around)
        area += points[i].x * points[j].y;
        area -= points[i].y * points[j].x;
    }

    area = std::abs(area) / 2.0;
    return area;
}


void to_json(json& j, const std::pair<int, double>& p) {
    j = json{{"to_id", p.first}, {"cost", p.second}};
}

void from_json(const json& j, std::pair<int, double>& p) {
    j.at("to_id").get_to(p.first);
    j.at("cost").get_to(p.second);
}



void to_json(json& j, const Node& node) {
    j = json{{"id", node.id}, {"x", node.x}, {"y", node.y}, {"edges", node.edges}};
}

// Convert JSON to Node
void Graph::from_json(const json& j, Node& node) {
    j.at("id").get_to(node.id);
    j.at("x").get_to(node.x);
    j.at("y").get_to(node.y);
    j.at("edges").get_to(node.edges);
}

// Export the nodes map to a JSON file
void Graph::save_nodes(const std::unordered_map<int, Node>& nodes, const std::string& filename) {
    json j_nodes;
    for (const auto& [id, node] : nodes) {
        j_nodes[std::to_string(id)] = node;  // `to_json` will be called here
    }
    std::ofstream file(filename);
    file << j_nodes.dump(4);  // Pretty print the JSON with 4-space indentation
    file.close();
}

// Import the nodes map from a JSON file
std::unordered_map<int, Node> Graph::load_nodes(const std::string& filename) {
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



// Optionally, implement more methods as needed
