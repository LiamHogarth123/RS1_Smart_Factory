#include "laserProcessing.hpp"
 
laserProcessing::laserProcessing() : Node("laser_processing"), current_goal_index_(0), navigation_stopped_(false)
{
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
 
    while (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
    }
 
    // Define 3 goals
    geometry_msgs::msg::PoseStamped goal1;
    goal1.header.frame_id = "map";
    goal1.pose.position.x = 1.0;
    goal1.pose.position.y = 1.0;
    goal1.pose.orientation.w = 1.0;
 
    geometry_msgs::msg::PoseStamped goal2;
    goal2.header.frame_id = "map";
    goal2.pose.position.x = 2.0;
    goal2.pose.position.y = -1.0;
    goal2.pose.orientation.w = 1.0;
 
    geometry_msgs::msg::PoseStamped goal3;
    goal3.header.frame_id = "map";
    goal3.pose.position.x = -1.0;
    goal3.pose.position.y = 2.0;
    goal3.pose.orientation.w = 1.0;
 
    // Add goals to the vector
    goals_.push_back(goal1);
    goals_.push_back(goal2);
    goals_.push_back(goal3);
 
    // Start by sending the first goal
    send_next_goal();
}
 
void laserProcessing::send_goal(const geometry_msgs::msg::PoseStamped& goal_pose)
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;
 
    // Store the last goal that was sent
    last_goal_ = goal_pose;
 
    RCLCPP_INFO(this->get_logger(), "Sending goal to (%.2f, %.2f)", goal_pose.pose.position.x, goal_pose.pose.position.y);
 
    // Define SendGoalOptions and result callback
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
    send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
        this->result_callback(result);
    };
 
    client_->async_send_goal(goal_msg, send_goal_options);
}
 
void laserProcessing::send_next_goal()
{
    if (current_goal_index_ < goals_.size()) {
        send_goal(goals_[current_goal_index_]);
    } else {
        RCLCPP_INFO(this->get_logger(), "All goals have been reached");
    }
}
 
void laserProcessing::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal %zu succeeded", current_goal_index_ + 1);
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(this->get_logger(), "Goal %zu was aborted", current_goal_index_ + 1);
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(this->get_logger(), "Goal %zu was canceled", current_goal_index_ + 1);
    }
 
    // Increment the goal index and send the next goal if any
    current_goal_index_++;
    send_next_goal();
}
 
void laserProcessing::stop_navigation()
{
    RCLCPP_INFO(this->get_logger(), "Stopping the current navigation goal...");
 
    // Cancel all goals currently being processed
    client_->async_cancel_all_goals();
 
    // Set the flag to true
    navigation_stopped_ = true;
}
 
void laserProcessing::restart_navigation()
{
    if (navigation_stopped_) {
        RCLCPP_INFO(this->get_logger(), "Restarting navigation to the last goal...");
 
        // Resend the last goal that was stopped
        send_goal(last_goal_);
 
        // Reset the stopped flag
        navigation_stopped_ = false;
    } else {
        RCLCPP_WARN(this->get_logger(), "Navigation is not currently stopped. Nothing to restart.");
    }
}
 