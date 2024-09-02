#include "turtlebot.hpp"

DefaultTurtleBot::DefaultTurtleBot(const std::string& name) {    
    // Create publishers and subscribers using the shared node
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(namespace_ + "/cmd_vel", 10);

    sub1_ = node_->create_subscription<nav_msgs::msg::Odometry>(namespace_ + "/odom", 10, std::bind(&DefaultTurtleBot::odomCallback, this, std::placeholders::_1));

    sub2_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(namespace_ + "/scan", 10, std::bind(&DefaultTurtleBot::LidaCallback, this, std::placeholders::_1));

    sub3_ = node_->create_subscription<sensor_msgs::msg::Image>(namespace_ + "/camera/rgb/image_raw", 10, std::bind(&DefaultTurtleBot::RGBCallback, this, std::placeholders::_1));

    sub4_ = node_->create_subscription<sensor_msgs::msg::Image>(namespace_ + "/camera/depth/image_raw", 10, std::bind(&DefaultTurtleBot::ImageDepthCallback, this, std::placeholders::_1));

//   sub5_ = this->create_subscription<std_msgs::msg::Int16>("/boundary/detection", 10, std::bind(&DefaultTurtleBot::boundaryCallback, this, std::placeholders::_1));

//   sub6_ = this->create_subscription<marker_msgs::msg::Marker>("/markers/info", 10, std::bind(&DefaultTurtleBot::tagCallback, this, std::placeholders::_1));
}

void DefaultTurtleBot::SendCmdTb1(const geometry_msgs::msg::Twist instructions){
  cmd_vel_pub_->publish(instructions);
} 

nav_msgs::msg::Odometry DefaultTurtleBot::GetCurrentOdom(){
  std::lock_guard<std::mutex> lock(odom_locker_);
  return current_odom_;
}

sensor_msgs::msg::Image DefaultTurtleBot::GetCurrentUpdatedRGB(){
  std::lock_guard<std::mutex> lock(RGB_locker_);
  return updated_rgb_;
}

sensor_msgs::msg::LaserScan DefaultTurtleBot::GetUpdatedLida(){
  std::lock_guard<std::mutex> lock(Lida_locker_);
  return updated_lida_;
}

sensor_msgs::msg::Image DefaultTurtleBot::GetUpdatedImageDepth(){
  std::lock_guard<std::mutex> lock(ImageDepth_locker_);
  return updated_image_depth_;
}

double DefaultTurtleBot::GetCurrentSpeed(){
  std::lock_guard<std::mutex> lock(odom_locker_);
  return current_speed_;
}

marker_msgs::msg::Marker DefaultTurtleBot::GetARTag(){
  std::lock_guard<std::mutex> lock(marker_locker_);
  return ar_tag_;
}

// std_msgs::msg::Int16 DefaultTurtleBot::GetBoundaryStatus(){
//   std::lock_guard<std::mutex> lock(boundary_locker_);
//   return boundary_status_;
// }

// Callbacks

void DefaultTurtleBot::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
  std::lock_guard<std::mutex> lock(odom_locker_);
  current_odom_ = *odomMsg;
  current_speed_ = std::sqrt(std::pow(odomMsg->twist.twist.linear.x, 2) +
                               std::pow(odomMsg->twist.twist.linear.y, 2) +
                               std::pow(odomMsg->twist.twist.linear.z, 2));
}

void DefaultTurtleBot::RGBCallback(const sensor_msgs::msg::Image::SharedPtr Msg){
  std::lock_guard<std::mutex> lock(RGB_locker_);
  updated_rgb_ = *Msg;
}

void DefaultTurtleBot::LidaCallback(const sensor_msgs::msg::LaserScan::SharedPtr Msg){
  std::lock_guard<std::mutex> lock(Lida_locker_);
  updated_lida_ = *Msg;
}

void DefaultTurtleBot::ImageDepthCallback(const sensor_msgs::msg::Image::SharedPtr Msg){
  std::lock_guard<std::mutex> lock(ImageDepth_locker_);
  updated_image_depth_ = *Msg;
}

void DefaultTurtleBot::guiderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg){
  std::lock_guard<std::mutex> lock(odom_locker2_);
  guider_odom_ = *odomMsg;
}

// void DefaultTurtleBot::boundaryCallback(const std_msgs::msg::Int16::SharedPtr Msg){
//   std::lock_guard<std::mutex> lock(boundary_locker_);
//   boundary_status_ = *Msg;
// }

// void DefaultTurtleBot::tagCallback(const marker_msgs::msg::Marker::SharedPtr Msg){
//   std::lock_guard<std::mutex> lock(marker_locker_);
//   ar_tag_ = *Msg;
// }