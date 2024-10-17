// Graph.h

#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <unordered_map>
#include <utility> // For std::pair
#include <string>
#include "opencv2/opencv.hpp"

#include "nav_msgs/msg/map_meta_data.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <random>
#include <algorithm>
#include <functional>
#include <queue>
#include <limits>

#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

// Define the structure for a node in the graph
struct Node {
    int id; // Unique identifier for the node
    double x, y; // Coordinates of the node in 2D space
    std::vector<std::pair<int, double>> edges; // List of edges (connected node ID, cost)
    Node() : id(-1), x(0.0), y(0.0) {}
    // Parameterized constructor
    Node(int id, double x, double y) : id(id), x(x), y(y) {}
};

struct Polygons {
    std::vector<cv::Point> Polyon_Points; // Unique identifier for the node
    int direction; // Coordinates of the node in 2D space
    Polygons(std::vector<cv::Point> Polyon_Points, int direction) : Polyon_Points(Polyon_Points), direction(direction) {}
};




// Define the Graph class to hold the nodes and provide methods to manipulate the graph
class Graph {
public:


    Graph(); // add constructor


    double heuristic(const Node& a, const Node& b);

    std::vector<int> a_star(const std::unordered_map<int, Node>& nodes, int start_id, int finish_id);


    void to_json(json& j, const std::pair<int, double>& p);

    void from_json(const json& j, std::pair<int, double>& p);



    void UpdateMapData(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData MapMetaData_);

    bool isInsidePolygon(const std::vector<cv::Point>& polygon, double x, double y);


    void prm_method();

    void Generate_Map();
    
    void generateGridNodes(const nav_msgs::msg::OccupancyGrid msg, const std::vector<Polygons> polygons);
    void generate_Prm_Nodes();

    void Click_Draw_path();
    
    void show_Prm(std::vector<int> path);
    cv::Mat Load_Map();
    cv::Mat visalise_prm(cv::Mat mapImage, std::unordered_map<int, Node> graph_struct);
    void show_map(cv::Mat mapImage);
    void save_map(cv::Mat mapImage);
    cv::Mat visalise_path(cv::Mat mapImage, std::vector<int> path);


    bool validate_point(geometry_msgs::msg::Point point);

    int find_node_at_cordinates(int x, int y);

    int find_closest_node_at_cordinates(int x, int y);

    void User_path_drawing(const cv::Mat mapImage);



    double angle_between_nodes(int x1, int y1, int x2, int y2);
    double distance_between_nodes(int x1, int y1, int x2, int y2);
    double normalise_angle(double angle);

    bool Is_Point_In_boundary(cv::Point point);

    bool Is_Point_In_FreeSpace(geometry_msgs::msg::Point point);




    std::unordered_map<int, Node> nodes; // A hashmap for fast lookup of nodes by ID

    void addNode(int id, double x, double y);
    void addEdge(int fromId, int toId, double cost);


    // Open CV generating Nodes
    std::vector<cv::Point> polygonPoints;
    static void staticMouseCallback(int event, int x, int y, int flags, void* userdata);
    std::vector<Polygons> getUserDefinedPolygons(const std::string& mapImagePath);

    cv::Mat drawPolygon(cv::Mat image);
    std::vector<std::pair<int, int>> bresenhamLinePoints(int startX, int startY, int endX, int endY);

    void createNodesAndEdges(int start_ID, int end_ID);

    bool pathIsClear(Node Node_A, Node Node_B);


    double calculate_polygon_area(const std::vector<cv::Point>& points);

    void to_json(json& j, const Node& node);
    // Convert JSON to Node
    void from_json(const json& j, Node& node);

    void save_nodes(const std::unordered_map<int, Node>& nodes, const std::string& filename);

    std::unordered_map<int, Node> load_nodes(const std::string& filename);

    
    // Optionally, declare more methods to remove nodes, edges, and other utilities
private:
    nav_msgs::msg::OccupancyGrid SlamMapData;
    int numberOfPoints_;
    nav_msgs::msg::MapMetaData latestMapMetaData_;
    
    std::vector<cv::Point> clicked_points;
    std::vector<cv::Point> currentPolygon;
    std::vector<cv::Point> boundary_Polygon;
    std::vector<Polygons> Lane_polygons;


    bool callback;

    int Id;




};

#endif // GRAPH_H
