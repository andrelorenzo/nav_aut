
#include <rclcpp/rclcpp.hpp>
#include <msg_srv_hunter/msg/marker.hpp>
#include <msg_srv_hunter/msg/polygon.hpp>
#include <msg_srv_hunter/msg/rectangle.hpp>
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <jsoncpp/json/json.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <robot_localization/srv/from_ll.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "area_path_gen.hpp"
#include <math.h>

class String2ros: public rclcpp::Node
{
    public:
        String2ros();
        ~String2ros();
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_pos;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_pos;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_goal_array;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_gt_web;

        rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr client_ll;

        rclcpp::TimerBase::SharedPtr timer_;

        msg_srv_hunter::msg::Marker marker;
        msg_srv_hunter::msg::Rectangle rectangle;
        msg_srv_hunter::msg::Polygon poly;
        geometry_msgs::msg::PoseStamped goal_pose;
        geometry_msgs::msg::PoseArray goal_pose_array;
        geometry_msgs::msg::PoseStamped robot_pose;
        geometry_msgs::msg::Point map_point;
        std_msgs::msg::String json;
        msg_srv_hunter::msg::Marker cord_past;
        msg_srv_hunter::msg::Marker cord;
        AreaPathGenerator path_generator;
        float step;
        float error;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void posCallback(const std_msgs::msg::String::SharedPtr msg);
        void fromllCallback(rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future);
        void send_request(msg_srv_hunter::msg::Marker marker);
        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void publishMessage();

};