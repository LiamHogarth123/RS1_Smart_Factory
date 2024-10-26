#ifndef PRM_HPP
#define PRM_HPP

#include <vector>
#include <utility> // for std::pair
#include <unordered_set>
#include <string>
#include "opencv2/opencv.hpp"

#include "nav_msgs/msg/map_meta_data.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <nlohmann/json.hpp>
#include <fstream>


using json = nlohmann::json;


// Node structure definition
struct Node {
    int id; // Unique identifier for the node
    double x, y; // Coordinates of the node in 2D space
    std::vector<std::pair<int, double>> edges; // List of edges (connected node ID, cost)
    Node() : id(-1), x(0.0), y(0.0) {}
    // Parameterized constructor
    Node(int id, double x, double y) : id(id), x(x), y(y) {}
};



struct ComparePair {
    bool operator()(const std::pair<float, int>& a, const std::pair<float, int>& b) const {
        return a.first > b.first; // Example: Min-heap based on the first element
    }
};

// PRM data structure definition
struct PrmData {
    std::vector<Node> Exported_Graph;
    nav_msgs::msg::OccupancyGrid map;
    nav_msgs::msg::MapMetaData MapMetaData_;
};

// PRM class definition
class Path_Planner {
public:
    Path_Planner(); 
    


    double heuristic(const Node& a, const Node& b);
    void import_map();
    void from_json(const json& j, Node& node);



    std::unordered_map<int, Node> nodes_Graph; // A hashmap for fast lookup of nodes by ID


    cv::Mat Load_Map();  // Function declaration
    // cv::Mat visalise_prm(cv::Mat mapImage, std::unordered_map<int, Node> graph_struct);
    void show_map();
    void save_map(cv::Mat mapImage);
    cv::Mat visalise_path(cv::Mat mapImage, std::vector<int> path);
    cv::Mat visalise_prm(cv::Mat mapImage, std::unordered_map<int, Node> graph_struct);




// Import the nodes map from a JSON file
    std::unordered_map<int, Node> load_nodes(const std::string& filename);
    // User Functions
    
    std::vector<geometry_msgs::msg::Point> A_star_To_Goal(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal);
    std::vector<geometry_msgs::msg::Point> A_star_To_Goal_With_Blacklist(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal, std::vector<geometry_msgs::msg::Point> collisionPoints);
    void UpdateMapData(nav_msgs::msg::OccupancyGrid map);
    std::vector<geometry_msgs::msg::Point> test();
    
    void show_Prm();
    PrmData ExportPrmData();
    void Load_PRM(PrmData imports);
    void User_remove_Nodes();
    bool checkCollision(const std::vector<geometry_msgs::msg::Point>& traj1, const std::vector<geometry_msgs::msg::Point>& traj2);
    



private:
    // PRM Generation Functions
   
    // Coordinate Conversion Functions
    geometry_msgs::msg::Point convertNodeToPoint(Node temp);
    float nodeDistance(const Node& a, const Node& b);
    float euclideanDistance(const Node& node1, const Node& node2);
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    int setGoalNode(geometry_msgs::msg::Point goal);
    std::vector<geometry_msgs::msg::Point> ConvertParthToWorld(std::vector<int> path, std::vector<Node> graph_);
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);
    std::vector<Node> rotateNodes(std::vector<Node>& graph, const geometry_msgs::msg::Quaternion& orientation, const geometry_msgs::msg::Pose& mapOrigin);
    bool newPoint(geometry_msgs::msg::Point point, std::vector<Node> temp);
    geometry_msgs::msg::Point convertPointToNodeCordinate(geometry_msgs::msg::Point temp);
    cv::Point convertPointToNodeCordinate(cv::Point temp);

    // Pathfinding
    std::vector<int> findPathAStar(const std::unordered_map<int, Node>& map_nodes, int startId, int targetId);
    std::vector<int> findPathAStarWithBlackList(const std::vector<Node>& graph, int startId, int targetId, const std::unordered_set<int>& blacklist);
    std::vector<int> findPathDijkstra(const std::vector<Node>& graph, int startId, int targetId);
    // void findPath(int startNodeId, int goalNodeId);
    std::unordered_set<int> GenerateBlackList(std::vector<geometry_msgs::msg::Point> collisionPoints);
    int findClosestNode(const geometry_msgs::msg::Point& point);
    // std::vector<int> getNodesInArea(int nodeId, int areaSize);

    // Visualization Functions
 

    // OpenCV Interaction
  

    // Node Removal Functions
    static void staticRemoveNodeCallback(int event, int x, int y, int flags, void* userdata);
    void removeNodeCallback(int event, int x, int y, int flags, void* userdata);
    cv::Mat removeNodes(cv::Mat mapImage, std::vector<Node>& graph_);

private:
    std::vector<Node> nodes;
    std::vector<Node> Graph;
    nav_msgs::msg::OccupancyGrid SlamMapData;
    nav_msgs::msg::MapMetaData latestMapMetaData_;
    double offsetX;
    double offsetY;
    double offsetYaw;
    int numberOfPoints_;
    std::vector<cv::Point> path_points_withoutValidation;
    std::vector<cv::Point> path_points;
    std::vector<int> path_id;
    std::string map_file_path;
    std::string saved_nodes_path;
    std::string save_map_file_path;
};

#endif // PRM_HPP
