
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/time.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <math.h>



class nav2commander : public rclcpp::Node
{
    using Nav2Pose = nav2_msgs::action::NavigateToPose;
    using Followaypoint = nav2_msgs::action::FollowWaypoints;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<Nav2Pose>;
    using GoalHandleFollowWaypoint = rclcpp_action::ClientGoalHandle<Followaypoint>;

    public:
        nav2commander();
        ~nav2commander();

    private:

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_single_goal_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_array_goals_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_follow_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_updated_goal_;
        rclcpp_action::Client<Nav2Pose>::SharedPtr navigate_to_pose_client_ptr_;
        rclcpp_action::Client<Followaypoint>::SharedPtr waypoint_follower_client_ptr_;

        //Methods
        void single_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void followCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void feedback_nav2pose_callback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const Nav2Pose::Feedback> feedback);
        void result_nav2pose_callback(const GoalHandleNavigateToPose::WrappedResult & result);
        void feedback_wayFollow_callback(
            GoalHandleFollowWaypoint::SharedPtr,
            const std::shared_ptr<const Followaypoint::Feedback> feedback);
        void result_wayFollow_callback(const GoalHandleFollowWaypoint::WrappedResult & result);

        //Variables
        bool follow_me;
        bool control;
};

