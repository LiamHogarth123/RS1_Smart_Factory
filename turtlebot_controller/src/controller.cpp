// Note I was unable to share a ros node between controller and turtlebot Ros2 mangers. Ideally combine these classes if performance is an issue

#include "controller.hpp"
#include <cmath>


Controller::Controller(const std::string &namespace_param)  : Node(namespace_param + "_controller"), namespace_param_(namespace_param) {


  // Initialize your classes
  Turtlebot_SensorProcessing machine_vision_;
  turtlebot_control Turtlebot_GPS_;
  NewPath_ = false;
  shutdown_request_ = false;

  // custom topics
  // Use relative topic names to avoid double namespace
  std::string path_topic = "trajectory"; 

  // Subscribe to the trajectory topic
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(path_topic, 10, std::bind(&Controller::pathCallback, this, std::placeholders::_1));
  goal_pub_ = this->create_publisher<std_msgs::msg::Bool>("reached_goal", 10);
  shut_down_request = this->create_subscription<std_msgs::msg::Bool>("shut_down", 10, std::bind(&Controller::shut_downCallback, this, std::placeholders::_1));\



  // Turtlebot topics
  // Publish to the cmd_vel topic
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  robot_info_pub = this->create_publisher<warehouse_robot_msgs::msg::RobotData>("robot_data", 10);


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


  while (active){
      
    //publish odom, status, and Ar Info if it is avaliable. 



    if (shutdown_request_){
      rclcpp::shutdown();
      active =false;
      break;
      rclcpp::shutdown();
    }
    rate.sleep();  // Sleep to prevent CPU hogging and allow callbacks to be processed


  }

  rclcpp::shutdown();

}


void Controller::controlLoop() {
  geometry_msgs::msg::Point goal;
  geometry_msgs::msg::Point target_goal;
  geometry_msgs::msg::Twist traj;
  bool goal_reached = false;
  std_msgs::msg::Bool status_msg;
  status_msg.data = false;  // Set the boolean value to true
  goal_pub_->publish(status_msg); 



  if (path_ == nullptr || path_->poses.empty()) {
    return;
  }

    
  goal = path_->poses.at(0).pose.position;
  nav_msgs::msg::Path trajectory_path = *path_;


  // firt aling the robots orination with first goal
  double Desired_yaw = Calculate_desired_yaw(trajectory_path);

  while (calculateYaw(current_odom_) - Desired_yaw > 0.5) {
    traj.angular.x = 0.1; // optimise this so direction and speed is considered
    SendCmdTb1(traj);
  }

  //stop the rotation
  SendCmdTb1(zero_trajectory);
  traj = zero_trajectory;
  
 

  //Once aligned start driving loop
  ////////////////////////////////////////////////////////////////////////
  while (!goal_reached){ 

    //find the current waypoint goal
    target_goal = findLookAheadPoint(trajectory_path, current_odom_.pose.pose.position, 0.5);
    //setups PID calculation
    Turtlebot_GPS_.updateControlParam(target_goal, 0.1, current_odom_);

    //If final Goal hit stop
    if (Turtlebot_GPS_.goal_hit(trajectory_path.poses.back().pose.position, current_odom_)){
      // end of goals reached
      goal_reached = true;
      break;
    }


  //if all good
  ///////////////////////////////////////////////////////////////////////////////////////////
  traj = Turtlebot_GPS_.generate_trajectory();
  SendCmdTb1(traj);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  }

  while (current_speed_ > 0){
    SendCmdTb1(zero_trajectory);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  }


  //rotat the turtlebot looking for the AR tag information
  // rotate turtleobot -90 degrees
  // double Max_angle = 90;
  // std::vector<int> found_tags;
  // for (angle = -90; angle < Max_angle; angle+10){
  //    found_tags = AR_TAG_DECTION_OBJECT.DECTECT_AR_FUNCTION(CURRENT_TURTLEBOT_IMAGE) 
  //    if (!found_tags.empty()){
  //        publish(AR_Info);
  //        break; 
  //    }
  // }
  




  status_msg.data = true;  // Set the boolean value to true
  Publish_robot_data(current_odom_, 0, 0);
  Publish_robot_data(current_odom_, 0, 0);
 
  NewPath_ = false;
}





geometry_msgs::msg::Point Controller::findLookAheadPoint(nav_msgs::msg::Path path, geometry_msgs::msg::Point Current_position, double tolerance){
  geometry_msgs::msg::Point lookahead_point = Current_position;

  // Iterate through the path points
  for (const auto& pose_stamped : path.poses) {
    geometry_msgs::msg::Point path_point = pose_stamped.pose.position;

    // Calculate the distance between the current position and the path point
    double distance_to_point = calculateDistance(Current_position, path_point);

    // If the distance is greater than the tolerance, this is our lookahead point
    if (distance_to_point > tolerance) {
      lookahead_point = path_point;
      break;
    }
  }

  // If we are near the end point (within the tolerance), set the lookahead point to the end of the path
  geometry_msgs::msg::Point end_point = path.poses.back().pose.position;
  double distance_to_end = calculateDistance(Current_position, end_point);
  if (distance_to_end <= tolerance) {
    lookahead_point = end_point;
  }
  return lookahead_point;

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
  msg.ar_tag_id = Ar_tag_info;
  robot_info_pub->publish(msg);
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
  controlLoop();
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
  double path_yaw;

  // complete calculations

  return path_yaw;
}


double Controller::calculateDistance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2) {
  return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
}

