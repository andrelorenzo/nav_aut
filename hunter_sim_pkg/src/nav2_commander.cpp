#include "hunter_sim_pkg/nav2_commander.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

nav2commander::nav2commander() : Node("Nav2Commander")
{
    this->declare_parameter("single_goal_topic","/commander/goal");
    const std::string single_goal = this->get_parameter("single_goal_topic").as_string();
    sub_single_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        single_goal, 10, std::bind(&nav2commander::single_callback, this, _1));

    this->declare_parameter("array_goals_topic","/commander/goal_array");
    const std::string array_goals = this->get_parameter("array_goals_topic").as_string();
    sub_array_goals_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        array_goals, 10, std::bind(&nav2commander::array_callback, this, _1));

    this->declare_parameter("follow_flag_topic","/web/follow");
    const std::string follow_flag = this->get_parameter("follow_flag_topic").as_string();
    sub_follow_ = this->create_subscription<std_msgs::msg::Bool>(
        follow_flag, 10, std::bind(&nav2commander::followCallback, this, _1));

    pub_updated_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_update", 10);    

    navigate_to_pose_client_ptr_ = rclcpp_action::create_client<Nav2Pose>(
        this, "navigate_to_pose");
    
    waypoint_follower_client_ptr_ = rclcpp_action::create_client<Followaypoint>(
        this, "follow_waypoints");

    follow_me = false;
    control = false;
    RCLCPP_INFO(this->get_logger(), "Ready for waypoints...");
}

nav2commander::~nav2commander(){}


void nav2commander::followCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    follow_me = (msg->data)?true:false;
    RCLCPP_DEBUG(this->get_logger(), "follow me changed to: %s",std::to_string(follow_me).c_str());
}


void nav2commander::single_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    while(!this->navigate_to_pose_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server(navigate to pose), not available after waiting");
    }
    auto send_goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
    send_goal_options.feedback_callback =std::bind(&nav2commander::feedback_nav2pose_callback, this, _1, _2);
    send_goal_options.result_callback =std::bind(&nav2commander::result_nav2pose_callback, this, _1);
    if( follow_me && !control)
    {
        control = true; 

        auto goal_msg = Nav2Pose::Goal();
        goal_msg.behavior_tree = "/home/andrelorent/tfg_ws/src/hunter_sim_pkg/config/follow_dynamic_waypoint.xml";
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position = msg->pose.position;
        goal_msg.pose.pose.orientation = msg->pose.orientation;


        RCLCPP_INFO(this->get_logger(), "Sending follow dynamic waypoint petition to [x: %.4f,y: %.4f]",goal_msg.pose.pose.position.x,goal_msg.pose.pose.position.y);

        auto goal_handle_future = navigate_to_pose_client_ptr_->async_send_goal(
            goal_msg, send_goal_options);
    }else if( follow_me && control)
    {
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.pose.position = msg->pose.position;
        goal_msg.pose.orientation = msg->pose.orientation;
        goal_msg.header.stamp = now();
        goal_msg.header.frame_id = "map";
        pub_updated_goal_->publish(goal_msg);
    }else{
        control = false;

        auto goal_msg = Nav2Pose::Goal();
        goal_msg.behavior_tree = "";
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position = msg->pose.position;
        goal_msg.pose.pose.orientation = msg->pose.orientation;


        RCLCPP_INFO(this->get_logger(), "GoToPose petition to [x: %.4f,y: %.4f]",goal_msg.pose.pose.position.x,goal_msg.pose.pose.position.y);

        auto goal_handle_future = navigate_to_pose_client_ptr_->async_send_goal(
            goal_msg, send_goal_options);

    }
}

void nav2commander::array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    while(!this->waypoint_follower_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server (Waypoint follower), not available after waiting");
    }
    auto send_goal_options = rclcpp_action::Client<Followaypoint>::SendGoalOptions();
    send_goal_options.feedback_callback =std::bind(&nav2commander::feedback_wayFollow_callback, this, _1, _2);
    send_goal_options.result_callback =std::bind(&nav2commander::result_wayFollow_callback, this, _1);

    auto goal_array_msg = Followaypoint::Goal();
    goal_array_msg.poses = {};
    for (geometry_msgs::msg::Pose p : msg->poses)
    {
        geometry_msgs::msg::PoseStamped p_stamped;
        p_stamped.pose = p;
        p_stamped.header.frame_id = "map";
        p_stamped.header.stamp = now();
        goal_array_msg.poses.push_back(p_stamped);
    }
    RCLCPP_INFO(this->get_logger(), "Sending %li waypoints to Waypoint follower",goal_array_msg.poses.size());

    auto goal_handle_future = waypoint_follower_client_ptr_->async_send_goal(
            goal_array_msg, send_goal_options);
    
}

void nav2commander::feedback_nav2pose_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const Nav2Pose::Feedback> feedback)
{
    if(fmod(feedback->distance_remaining,1.0) == 0){
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f",
      feedback->distance_remaining);
    }

}

void nav2commander::result_nav2pose_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!!!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
}

void nav2commander::feedback_wayFollow_callback(
    GoalHandleFollowWaypoint::SharedPtr,
    const std::shared_ptr<const Followaypoint::Feedback> feedback)
{
    static int waypoint = 1;
    if(waypoint != feedback->current_waypoint + 1){
    RCLCPP_INFO(get_logger(), "executing waypoint:  %i",
      feedback->current_waypoint + 1);
      waypoint = feedback->current_waypoint + 1;
    }
}

void nav2commander::result_wayFollow_callback(const GoalHandleFollowWaypoint::WrappedResult & result)
{
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!!!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
}



int main ( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init ( argc, argv );
    // Create object from our datumGen class
    auto node = std::make_shared<nav2commander>();
    // run at 30Hz
    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);    // attend subscriptions and srv request
        loop_rate.sleep();          // sleep till next step time
    }

    rclcpp::shutdown();
    return 0;
}
