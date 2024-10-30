// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "controller.hpp"
#include <cmath>


Controller::Controller(const std::string &namespace_param)  : Node(namespace_param + "_controller"), namespace_param_(namespace_param) {


  // Initialize your classes
  // Turtlebot_SensorProcessing machine_vision_;
  turtlebot_control Turtlebot_GPS_;
  NewPath_ = false;
  shutdown_request_ = false;

  // custom topics
  // Use relative topic names to avoid double namespace
  std::string path_topic = "trajectory"; 

  // Subscribe to the trajectory topic
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(path_topic, 10, std::bind(&Controller::pathCallback, this, std::placeholders::_1));
  goal_pub_ = this->create_publisher<std_msgs::msg::Bool>("reached_goal", 10);
  custom_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("backup/odom", 10);


  shut_down_request = this->create_subscription<std_msgs::msg::Bool>("shut_down", 10, std::bind(&Controller::shut_downCallback, this, std::placeholders::_1));\



  // Turtlebot topics
  // Publish to the cmd_vel topic
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  robot_info_pub = this->create_publisher<warehouse_robot_msgs::msg::RobotData>("robot_data", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);


  // Subscribe to other topics without manually adding the namespace
  sub1_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Controller::odomCallback, this, std::placeholders::_1));
  sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Controller::LidaCallback, this, std::placeholders::_1));
  sub3_ = this->create_subscription<sensor_msgs::msg::Image>("camera/rgb/image_raw", 10, std::bind(&Controller::RGBCallback, this, std::placeholders::_1));
  sub4_ = this->create_subscription<sensor_msgs::msg::Image>("camera/depth/image_raw", 10, std::bind(&Controller::ImageDepthCallback, this, std::placeholders::_1));



  zero_trajectory.angular.x = 0; 
  zero_trajectory.angular.y = 0;
  zero_trajectory.angular.z = 0;
  zero_trajectory.linear.x = 0;
  zero_trajectory.linear.y = 0;

}







// DEFAULT STATE
//////////////////////////////////////////////////////
void Controller::Default_state(){

  rclcpp::Rate rate(10); // 10 Hz
  bool active = true;

  std::cout << "Entering Default State..." << std::endl;

  std::cout << "Publishing odom, status, and AR info (if available)..." << std::endl;

  while (active){
      
    //publish odom, status, and Ar Info if it is available.
    Publish_robot_data(current_odom_, 0, -1);
    Publish_custom_odom(current_odom_);

    if (NewPath_){
      std::cout << "New Path detected, entering control loop..." << std::endl;
      Publish_robot_data(current_odom_, 1, -1);

      for (size_t i = 0; i < path_->poses.size(); ++i) {
        double x = path_->poses[i].pose.position.x;
        double y = path_->poses[i].pose.position.y;
        double z = path_->poses[i].pose.position.z;

        std::cout << "Waypoint " << i << ": x=" << x << ", y=" << y << ", z=" << z << std::endl;
      }
      controlLoop();
    }
    if (shutdown_request_){
      std::cout << "Shutdown request received, shutting down..." << std::endl;
      rclcpp::shutdown();
      active = false;
      break;
    }
    rate.sleep();  // Sleep to prevent CPU hogging and allow callbacks to be processed
  }

  std::cout << "Exiting Default State, shutting down..." << std::endl;
  rclcpp::shutdown();
}


void Controller::controlLoop() {
  geometry_msgs::msg::Point goal;
  geometry_msgs::msg::Point target_goal;
  geometry_msgs::msg::Twist traj;
  bool goal_reached = false;
  std_msgs::msg::Bool status_msg;
  Publish_robot_data(current_odom_, 1, -1);
  Publish_robot_data(current_odom_, 1, -1);
  Publish_robot_data(current_odom_, 1, -1);
  Publish_robot_data(current_odom_, 1, -1);


  status_msg.data = false;  // Set the boolean value to false initially
  goal_pub_->publish(status_msg); 

  std::cout << "Entering control loop..." << std::endl;

  if (path_ == nullptr || path_->poses.empty()) {
    std::cout << "Path is null or empty, exiting control loop..." << std::endl;
    return;
  }

  std::cout << "Path found, proceeding with alignment..." << std::endl;
    
  goal = path_->poses.at(0).pose.position;
  nav_msgs::msg::Path trajectory_path = *path_;

  publishMarkerArray(trajectory_path);

  // first align the robot's orientation with the first goal
  double Desired_yaw = Calculate_desired_yaw(trajectory_path);

  std::cout << "Desired yaw: " << Desired_yaw << std::endl;
  std::cout << "current yaw" << std::endl;
  std::cout << "alligning yaw" << std::endl;

  while (Turtlebot_GPS_.angleToGoal(current_odom_, trajectory_path.poses.at(1).pose.position) > 0.1) {
    // std::cout << "Aligning robot: current yaw = " << calculateYaw(current_odom_) << std::endl;
    traj.angular.z = Turtlebot_GPS_.steeringPID(trajectory_path.poses.at(1).pose.position, current_odom_);  // optimize this so direction and speed is considered
           
    SendCmdTb1(traj);
    Publish_custom_odom(current_odom_);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "Robot aligned with the first goal." << std::endl;

  // Stop the rotation
  SendCmdTb1(zero_trajectory);
  SendCmdTb1(zero_trajectory);

  traj = zero_trajectory;

  // Once aligned, start driving loop
  ////////////////////////////////////////////////////////////////////////
  std::cout << "Starting driving loop..." << std::endl;
  while (!goal_reached){ 
    
    // Find the current waypoint goal
    target_goal = findLookAheadPoint(trajectory_path, current_odom_.pose.pose.position, 0.5);
    std::cout << "Lookahead point at (" << target_goal.x << ", " << target_goal.y << ")" << std::endl;

    publishSingleMarker(target_goal);

    // Setup PID calculation
    Turtlebot_GPS_.updateControlParam(target_goal, 0.1, current_odom_, updated_lida_);
    std::cout << "PID control updated with target goal." << std::endl;

    // If final goal is hit, stop
    if (Turtlebot_GPS_.goal_hit(trajectory_path.poses.back().pose.position, current_odom_)) {
      std::cout << "Final goal reached." << std::endl;
      goal_reached = true;
      break;
    }

    // If all good
    ///////////////////////////////////////////////////////////////////////////////////////////
    traj = Turtlebot_GPS_.generate_trajectory();
    std::cout << "Generated trajectory: linear.x = " << traj.linear.x << ", angular.z = " << traj.angular.z << std::endl;
    SendCmdTb1(traj);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }

  std::cout << "Stopping the TurtleBot, current speed: " << current_speed_ << std::endl;

  while (current_speed_ > 0.05){
    SendCmdTb1(zero_trajectory);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::cout << "Decelerating..." << std::endl;
  }
  SendCmdTb1(zero_trajectory);
  SendCmdTb1(zero_trajectory);


  std::cout << "TurtleBot stopped." << std::endl;

  // Rotate the TurtleBot looking for the AR tag information
  // rotate TurtleBot -90 degrees
  // double Max_angle = 90;
  // std::vector<int> found_tags;
  // for (angle = -90; angle < Max_angle; angle+10){
  //    found_tags = AR_TAG_DECTION_OBJECT.DECTECT_AR_FUNCTION(CURRENT_TURTLEBOT_IMAGE) 
  //    if (!found_tags.empty()){
  //        publish(AR_Info);
  //        break; 
  //    }
  // }

  std::cout << "Publishing final status and data..." << std::endl;

  status_msg.data = true;  // Set the boolean value to true
  Publish_robot_data(current_odom_, 0, 0);
  Publish_robot_data(current_odom_, 0, 0);
  Publish_custom_odom(current_odom_);

  NewPath_ = false;
}





geometry_msgs::msg::Point Controller::findLookAheadPoint(nav_msgs::msg::Path path, geometry_msgs::msg::Point current_position, double tolerance) {
  if (path.poses.empty()) {
    // Return the current position if the path is empty
    return current_position;
  }

  double closest_distance = std::numeric_limits<double>::max();
  size_t closest_point_id = 0;
  geometry_msgs::msg::Point look_ahead_point = path.poses.front().pose.position;

  // Find the closest point on the path to the current position
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double distance = calculateDistance(path.poses[i].pose.position, current_position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_point_id = i;
      look_ahead_point = path.poses[i].pose.position;
    }
  }

  // Check if the closest point is behind the TurtleBot, and if so, move to the next point
  if (closest_point_id + 1 < path.poses.size()) { // Ensure we are within bounds
    double dist_to_next = calculateDistance(path.poses[closest_point_id].pose.position, path.poses[closest_point_id + 1].pose.position);
    double dist_to_current = calculateDistance(path.poses[closest_point_id].pose.position, current_position);
    double dist_next_to_current = calculateDistance(path.poses[closest_point_id + 1].pose.position, current_position);

    // If current position is between the closest and next point, increment to the next point
    if (dist_to_next >= dist_next_to_current && dist_to_next >= dist_to_current) {
      closest_point_id++;
      look_ahead_point = path.poses[closest_point_id].pose.position;
    }
  }

  // Calculate the cumulative distance along the path starting from the current position
  double cumulative_distance = calculateDistance(current_position, path.poses[closest_point_id].pose.position);

  for (size_t i = closest_point_id; i < path.poses.size() - 1; ++i) {
    double segment_length = calculateDistance(path.poses[i].pose.position, path.poses[i + 1].pose.position);
    cumulative_distance += segment_length;

    // If the cumulative distance exceeds the tolerance, calculate the look-ahead point
    if (cumulative_distance >= tolerance) {
      double overshoot = cumulative_distance - tolerance;
      double ratio = (segment_length - overshoot) / segment_length;

      look_ahead_point.x = path.poses[i].pose.position.x + ratio * (path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x);
      look_ahead_point.y = path.poses[i].pose.position.y + ratio * (path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y);
      return look_ahead_point;
    }
  }

  // If tolerance not reached, return the last point in the path
  return path.poses.back().pose.position;
}

void Controller::shut_downCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    shutdown_request_ = true;
    rclcpp::shutdown();
  }
}





//Ros2 turtlebot control
///////////////////////////////////////////////////////////////////////////////////
void Controller::SendCmdTb1(const geometry_msgs::msg::Twist instructions){
  cmd_vel_pub_->publish(instructions);
} 


void Controller::Publish_robot_data(nav_msgs::msg::Odometry odom, int status, int Ar_tag_info) {
  warehouse_robot_msgs::msg::RobotData msg;
  msg.odom = odom;
  msg.status = status; 
  // msg.ar_tag_id = Ar_tag_info;
  robot_info_pub->publish(msg);
}

void Controller::Publish_custom_odom(nav_msgs::msg::Odometry odom){
  custom_odom_pub->publish(odom);

}

// Callbacks
///////////////////////////////////////////////////////////////////////////////////
void Controller::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
  std::lock_guard<std::mutex> lock(odom_locker_);
  current_odom_ = *odomMsg;
  current_speed_ = std::sqrt(std::pow(odomMsg->twist.twist.linear.x, 2) + std::pow(odomMsg->twist.twist.linear.y, 2) + std::pow(odomMsg->twist.twist.linear.z, 2));
}


void Controller::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  path_ = msg;
  current_waypoint_ = 0;
  NewPath_ = true;
  // controlLoop();
}

void Controller::RGBCallback(const sensor_msgs::msg::Image::SharedPtr Msg){
  std::lock_guard<std::mutex> lock(RGB_locker_);
  updated_rgb_ = *Msg;
}

void Controller::LidaCallback(const sensor_msgs::msg::LaserScan::SharedPtr Msg){
  std::lock_guard<std::mutex> lock(Lida_locker_);
  updated_lida_ = *Msg;
}

void Controller::ImageDepthCallback(const sensor_msgs::msg::Image::SharedPtr Msg){
  std::lock_guard<std::mutex> lock(ImageDepth_locker_);
  updated_image_depth_ = *Msg;
}

void Controller::guiderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg){
  std::lock_guard<std::mutex> lock(odom_locker2_);
  guider_odom_ = *odomMsg;
}



//Reused calculations
///////////////////////////////////////////////////////////////////////////////////
double Controller::calculateYaw(nav_msgs::msg::Odometry odometry_data){
  // Extract quaternion from odometry data
  double x = odometry_data.pose.pose.orientation.x;
  double y = odometry_data.pose.pose.orientation.y;
  double z = odometry_data.pose.pose.orientation.z;
  double w = odometry_data.pose.pose.orientation.w;

  // Convert quaternion to yaw (rotation around Z-axis)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return yaw; // Return yaw in radians
}

double Controller::Calculate_desired_yaw(nav_msgs::msg::Path path){
  double deltaX = path.poses.at(1).pose.position.x - current_odom_.pose.pose.position.x;
  double deltaY = path.poses.at(1).pose.position.y - current_odom_.pose.pose.position.y;
  
  // Calculate the angle (yaw) to face the point
  double required_yaw = atan2(deltaY, deltaX);

  return required_yaw;
}

double Controller::calculateDistance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2) {
  return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
}





// visualisation section
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Controller::publishMarkerArray(const nav_msgs::msg::Path &paths) {
   visualization_msgs::msg::MarkerArray marker_array;

    // Debug output: Check if the path contains any poses

    int marker_id = 0;
    for (const auto &pose_stamped : paths.poses) {

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Replace "map" with your desired frame
        marker.header.stamp = this->now();
        marker.ns = "path_marker_array";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker position to each point in the path
        marker.pose.position = pose_stamped.pose.position;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;



        // Set the marker geometry_msgs::msg::Point temp_goal, nav_msgs::msg::Odometry temp_odome
        marker.scale.x = 0.1;  // Adjust size as needed
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;


        // Set color (RGBA)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;



        // Add marker to the array
        marker_array.markers.push_back(marker);
    }

    // Publish the marker array
    marker_array_pub_->publish(marker_array);


}

void Controller::publishSingleMarker(const geometry_msgs::msg::Point &point) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Replace "map" with your frame of choice
        marker.header.stamp = this->now();
        marker.ns = "single_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker position
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;

        // Marker properties
        marker.scale.x = 0.2;  // Set size as needed
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set color (RGBA)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Publish the marker
        marker_pub_->publish(marker);
    }