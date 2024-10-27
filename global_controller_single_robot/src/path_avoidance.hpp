#include <vector>
#include <geometry_msgs/msg/point.hpp>

struct TurtleBotState {
    geometry_msgs::msg::Point position;
    double time;  // Timestamp in seconds
};

class path_avoidance_ {
public:
    path_avoidance_();

    void Update_paths(std::vector<std::vector<geometry_msgs::msg::Point>> trajectories);

    bool find_collisions(std::vector<geometry_msgs::msg::Point>& collided_points);

    std::vector<geometry_msgs::msg::Point> get_all_interpolated_points();

    std::vector<std::pair<int, geometry_msgs::msg::Point>> GetCollisionsWithId();

private:
    std::vector<std::vector<geometry_msgs::msg::Point>> trajectories;
    std::vector<std::vector<TurtleBotState>> interpolated_trajectories_;
    std::vector<std::pair<int, geometry_msgs::msg::Point>> collided_points_;

    std::vector<TurtleBotState> interpolatePositions(const std::vector<geometry_msgs::msg::Point>& waypoints, double average_speed);
    void interpolated_trajectories();
    bool will_collide();
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    bool checkCollisions(const std::vector<std::pair<int, geometry_msgs::msg::Point>>& points_with_ids, double threshold);
};