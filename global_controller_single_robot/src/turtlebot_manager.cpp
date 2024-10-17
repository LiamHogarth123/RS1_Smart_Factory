#include "turtlebot.hpp"

DefaultTurtleBot::DefaultTurtleBot(const std::string& name) {    
    // Create publishers and subscribers using the shared node
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(namespace_ + "/cmd_vel", 10);

    sub1_ = node_->create_subscription<nav_msgs::msg::Odometry>(namespace_ + "/robot_info", 10, std::bind(&DefaultTurtleBot::robot_info_Callback, this, std::placeholders::_1));

 
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);

//   sub5_ = this->create_subscription<std_msgs::msg::Int16>("/boundary/detection", 10, std::bind(&DefaultTurtleBot::boundaryCallback, this, std::placeholders::_1));

//   sub6_ = this->create_subscription<marker_msgs::msg::Marker>("/markers/info", 10, std::bind(&DefaultTurtleBot::tagCallback, this, std::placeholders::_1));
}

void DefaultTurtleBot::SendCmdTb1(const geometry_msgs::msg::Twist instructions){
  cmd_vel_pub_->publish(instructions);
} 

void DefaultTurtleBot::publishTrajectory(std::vector<geometry_msgs::msg::Point> goals){
    auto path_msg = std::make_shared<nav_msgs::msg::Path>();
    path_msg->header.stamp = this->get_clock()->now();
    path_msg->header.frame_id = "map"; // Replace with your frame of reference

    // Define five different (x, y) coordinates
    

    for (const auto& coord : goals) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map"; // Replace with your frame of reference
        pose.pose.position.x = coord.x;
        pose.pose.position.y = coord.y;
        pose.pose.position.z = 0.0; // Assuming a flat plane

        // You can add orientation if needed, here assuming no orientation
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        // Add the pose to the path
        path_msg->poses.push_back(pose);
    }

    // Publish the path message
    path_pub_->publish(*path_msg);
    robot_status = false;
    RCLCPP_INFO(this->get_logger(), "Published a trajectory with %zu points", path_msg->poses.size());

}




nav_msgs::msg::Odometry DefaultTurtleBot::GetCurrentOdom(){
  std::lock_guard<std::mutex> lock(odom_locker_);
  return current_odom_;
}

double DefaultTurtleBot::GetCurrentSpeed(){
  std::lock_guard<std::mutex> lock(odom_locker_);
  return current_speed_;
}

marker_msgs::msg::Marker DefaultTurtleBot::GetARTag(){
  std::lock_guard<std::mutex> lock(marker_locker_);
  return ar_tag_;
}

// // Get status_function(){
// return status
// }


// Callbacks
void DefaultTurtleBot::robot_info_Callback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
    std::lock_guard<std::mutex> lock(odom_locker_);
    current_odom_ = *odomMsg;
    current_speed_ = std::sqrt(std::pow(odomMsg->twist.twist.linear.x, 2) +
                               std::pow(odomMsg->twist.twist.linear.y, 2) +
                               std::pow(odomMsg->twist.twist.linear.z, 2));

    int status = 0;
    int Artag_info = 0;


}
