// Graph.cpp

#include "prm2.hpp"

Graph::Graph()
{
}

void Graph::UpdateMapData(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData MapMetaData_)
{
    std::cout << "PRM map data openning" << std::endl;
    SlamMapData = map;
    numberOfPoints_ = 10000;
    latestMapMetaData_ = MapMetaData_;
}

void Graph::Generate_Map()
{

    
    std::vector<Polygons> userPolygon = getUserDefinedPolygons("/home/liam/map.pgm");
    generateGridNodes(SlamMapData, userPolygon);
    // for (int x = 0; x < nodes.size(); x++) {
    //     std::cout << "Node " << x << ": (" << nodes[x].x << ", " << nodes[x].y << ")" << std::endl;
    // }

//    for (int j = 0; j < SlamMapData.data.size(); j++) {
//         std::cout << "grid[" << j << "] = " << static_cast<int>(SlamMapData.data.at(j)) << std::endl;
//     }

    show_Prm();
}

void Graph::show_Prm()
{
    std::cout << "Show PRM Opens" << std::endl;
    cv::Mat MapImage = Load_Map();
    MapImage = visalise_prm(MapImage, nodes);
    save_map(MapImage);
    show_map(MapImage);
}

void Graph::show_map(cv::Mat mapImage)
{
    cv::namedWindow("SLAM Map with Nodes", cv::WINDOW_AUTOSIZE);
    cv::imshow("SLAM Map with Nodes", mapImage);
}

void Graph::save_map(cv::Mat mapImage)
{
    cv::imwrite("/home/liam/Desktop/map_with_nodes fixed!!!.png", mapImage);
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

    cv::flip(mapImage, mapImage, 0);
    std::cout << "Flipped the map image vertically.\n";

    return mapImage;
}

cv::Mat Graph::visalise_prm(cv::Mat mapImage, std::unordered_map<int, Node> graph_struct)
{
    int radius = 5;              // Adjust this value as needed for the size of the nodes
    cv::Scalar node_color(255, 0, 0); // BGR value for red
    cv::Scalar edge_color(0, 255, 255); // BGR value for yellow

    // Step 1: Visualize Nodes
    std::cout << "Visualizing Nodes" << std::endl;
    for (const auto &[node_id, node] : graph_struct)
    {
        // Draw each node as a circle
        cv::Point center(node.x, -node.y); // Flip y-coordinate
        std::cout << "Node ID: " << node_id << " at (" << node.x << ", " << -node.y << ")" << std::endl;

        // Only draw the node if it's within image bounds
        if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows)
        {
            cv::circle(mapImage, center, radius, node_color, -1); // Draw the node as a filled circle
        }
    }

    // Step 2: Visualize Edges Separately
    std::cout << "Visualizing Edges" << std::endl;
    for (const auto &[node_id, node] : graph_struct)
    {
        cv::Point center(node.x, -node.y); // Center point of the current node

        for (const auto &edge : node.edges)
        {
            int connected_node_id = edge.first;
            const auto &connected_node = graph_struct.at(connected_node_id);

            cv::Point connected_node_center(connected_node.x, -connected_node.y); // Center point of the connected node

            // Only draw the edge if both nodes are within image bounds
            if (center.x >= 0 && center.x < mapImage.cols && center.y >= 0 && center.y < mapImage.rows &&
                connected_node_center.x >= 0 && connected_node_center.x < mapImage.cols &&
                connected_node_center.y >= 0 && connected_node_center.y < mapImage.rows)
            {
                std::cout << "Edge from Node (" << node_id << ") to Node (" << connected_node_id << ")" << std::endl;
                cv::line(mapImage, center, connected_node_center, edge_color, 2); // Draw the edge
            }
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

    int id = 0;
    int id_start;

    for (int c = 0; c < polygons.size(); c++){
        id_start = id;
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
                        addNode(id, x, y);
                        std::cout << "adding_node" << std::endl;
                        id++;
                    }
                }   
            }
        }
        std::cout << id_start << std::endl;
        std::cout << id << std::endl;
        int id_start_loop = id_start;
        for (int j = id_start; j < id; j++){
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



bool Graph::validate_point(geometry_msgs::msg::Point point){

    // Convert the point's x and y to grid indices
    int grid_x = static_cast<int>(point.x);
    int grid_y = static_cast<int>(point.y);

    // Calculate the linear index of the point on the occupancy grid
    int index = grid_x + (grid_y * SlamMapData.info.width);
    
    // Define the size of the grid to check around the point
    int grid_size = 4; 

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
                std::cout << "got Here" << std::endl;
                // lanes.push_back(currentPolygon);
                int direction;
                std::cout << "got to the next step" << std::endl;
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
                    default:
                        // key_pressed = true;
                        // std::cout << "Invalid key pressed." << std::endl;
                        break;
                    }

                }
                // directions.push_back(direction);
                Lanes.emplace_back(currentPolygon, 1);
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
        std::cout << "Point added: (" << x << ", " << y << ")" << std::endl;
    }
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
    // Check if node with the same id already exists
    if (nodes.find(id) == nodes.end())
    {
        nodes.emplace(id, Node(id, x, y));
    }
    else
    {
        // Optional: Handle the case where the node already exists
        std::cerr << "Node with id " << id << " already exists. Skipping addition." << std::endl;
    }
}

void Graph::addEdge(int fromId, int toId, double cost)
{
    nodes[fromId].edges.push_back(std::make_pair(toId, cost));
}




// Optionally, implement more methods as needed
