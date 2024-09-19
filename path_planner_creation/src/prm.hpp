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

// Node structure definition
struct Node {
    int id;
    float x, y; // Node position
    std::vector<int> edges; // IDs of connected nodes
};

// Custom comparison structure for priority queue
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
class PRM {
public:
    PRM(); 

    // User Functions
    void GeneratePRM(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData mapMetaData_, bool userControlled);
    std::vector<geometry_msgs::msg::Point> DijkstraToGoal(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal);
    std::vector<geometry_msgs::msg::Point> A_star_To_Goal(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal);
    std::vector<geometry_msgs::msg::Point> A_star_To_Goal_With_Blacklist(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal, std::vector<geometry_msgs::msg::Point> collisionPoints);
    void UpdateMapData(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData mapMetaData_);
    std::vector<geometry_msgs::msg::Point> test();
    void show_Prm();
    PrmData ExportPrmData();
    void Load_PRM(PrmData imports);
    void User_remove_Nodes();
    bool checkCollision(const std::vector<geometry_msgs::msg::Point>& traj1, const std::vector<geometry_msgs::msg::Point>& traj2);

    // void PRM::GeneratePRM(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::MapMetaData MapMetaData_, bool User_controlled);


private:
    // PRM Generation Functions
    std::vector<Node> samplePoints();
    bool isPointInLShape(const cv::Point& point);
    std::vector<cv::Point> LPoints;
    bool ValidPoint(geometry_msgs::msg::Point point);
    bool pathIsClear(Node NodeA, Node NodeB);
    std::vector<std::pair<int, int>> bresenhamLinePoints(int startX, int startY, int endX, int endY);
    bool ValidPointForPath(int x1, int y1);
    std::vector<Node> createNodesAndEdges(std::vector<Node> graph_);

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
    std::vector<int> findPathAStar(const std::vector<Node>& graph, int startId, int targetId);
    std::vector<int> findPathAStarWithBlackList(const std::vector<Node>& graph, int startId, int targetId, const std::unordered_set<int>& blacklist);
    std::vector<int> findPathDijkstra(const std::vector<Node>& graph, int startId, int targetId);
    void findPath(int startNodeId, int goalNodeId);
    std::unordered_set<int> GenerateBlackList(std::vector<geometry_msgs::msg::Point> collisionPoints);
    int findClosestNode(const geometry_msgs::msg::Point& point);
    std::vector<int> getNodesInArea(int nodeId, int areaSize);

    // Visualization Functions
    void visualizePRM(std::vector<Node> graph_, std::vector<int> path);
    cv::Mat Load_Map();
    cv::Mat visalise_prm(cv::Mat mapImage, std::vector<Node> graph_);
    cv::Mat visalise_PRM_With_Path(std::vector<int> path, cv::Mat mapImage, std::vector<Node> graph_);
    void show_map(cv::Mat mapImage);
    void save_map(cv::Mat mapImage);

    // OpenCV Interaction
    static void staticMouseCallback(int event, int x, int y, int flags, void* userdata);
    void mouseCallback(int event, int x, int y, int flags, void* userdata);
    std::vector<cv::Point> getUserDefinedPolygon(const std::string& mapImagePath);
    bool isPointInPolygon(const cv::Point& point, const std::vector<cv::Point>& polygon);
    std::vector<Node> samplePointsCV();
    static std::vector<cv::Point> polygonPoints;

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
};

#endif // PRM_HPP
