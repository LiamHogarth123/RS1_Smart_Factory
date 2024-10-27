#include "turtlebot_manager.hpp"

TurtleBotManager::TurtleBotManager(const std::string& name, int id) : Node(name + "turtlebot_manager_node" + std::to_string(id)), namespace_(name) {    

  data_recieved = false;
  odom_recieved = false;

  // Prefix the namespace to the topic names
  std::string ns_prefix = namespace_ + "/";

  // Create publisher with namespace
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(ns_prefix + "trajectory", 10);

  std::cout << "subsciber_created" << std::endl;

  // Create subscriptions with namespace
  back_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      ns_prefix + "backup/odom", 10, 
      std::bind(&TurtleBotManager::customOdomCallback, this, std::placeholders::_1));

  robot_info_sub = this->create_subscription<warehouse_robot_msgs::msg::RobotData>(
        ns_prefix + "robot_data", 
        10, 
        std::bind(static_cast<void(TurtleBotManager::*)(const warehouse_robot_msgs::msg::RobotData::SharedPtr)>(&TurtleBotManager::robot_info_Callback), 
                  this, std::placeholders::_1));



}



void TurtleBotManager::publishTrajectory(std::vector<geometry_msgs::msg::Point> goals){
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

      // Add the pose to the pathlock(odom_locker_);
//     current_odom_ = *odomMsg.odom;
//     current_speed_ = std::sqrt(std::
      path_msg->poses.push_back(pose);
  }

  // Publish the path message
  path_pub_->publish(*path_msg);
  // robot_status = false;
  RCLCPP_INFO(this->get_logger(), "Published a trajectory with %zu points", path_msg->poses.size());

}




nav_msgs::msg::Odometry TurtleBotManager::GetCurrentOdom(){


  std::lock_guard<std::mutex> lock(odom_locker_);
  return current_odom_;
}

double TurtleBotManager::GetCurrentSpeed(){
  std::lock_guard<std::mutex> lock(odom_locker_);
  return current_speed_;
}

int TurtleBotManager::GetARTag(){
  return AR_tag;
}

std::string TurtleBotManager::getStatus() {
  std::string status_string;
  if (status == 0){
    status_string = "waiting";
  }
  if (status == 1){
    status_string = "running";
  }
  if (status == 2){
    status_string = "error";
  }
  return status_string;
}

bool TurtleBotManager::get_status_bool(){
  if (status == 1){
    return false;
  }
  else {
    return true;
  }
}



void TurtleBotManager::robot_info_Callback(const warehouse_robot_msgs::msg::RobotData::SharedPtr msg) {
    // Process the RobotData message here
  // std::cout << "got robot_info" << std::endl;
  data_recieved = true;
  odom_recieved = true;
  

  // std::lock_guard<std::mutex> lock(odom_locker_);
  current_odom_ = msg->odom;
  current_speed_ = std::sqrt(std::pow(current_odom_.twist.twist.linear.x, 2) +
                              std::pow(current_odom_.twist.twist.linear.y, 2) +
                              std::pow(current_odom_.twist.twist.linear.z, 2));

  status = msg->status;
  int Artag_info = msg->ar_tag_id;
  


}

// Callbacks
// void DefaultTurtleBot::robot_info_Callback(const warehouse_robot_msgs::msg::RobotData::SharedPtr odomMsg) {
//     std::lock_guard<std::mutex> lock(odom_locker_);
//     current_odom_ = *odomMsg.odom;
//     current_speed_ = std::sqrt(std::pow(current_odom_->twist.twist.linear.x, 2) +
//                                std::pow(current_odom_->twist.twist.linear.y, 2) +
//                                std::pow(current_odom_->twist.twist.linear.z, 2));

//     int status = odomMsg.status;
//     int Artag_info = odomMsg.status;


// }


void TurtleBotManager::customOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
  // std::lock_guard<std::mutex> lock(odom_locker_);
  // std::cout << "got Odom_second topic" << std::endl;
  odom_recieved = true;
  current_odom_ = *odomMsg;
  current_speed_ = std::sqrt(std::pow(odomMsg->twist.twist.linear.x, 2) + std::pow(odomMsg->twist.twist.linear.y, 2) + std::pow(odomMsg->twist.twist.linear.z, 2));
}