#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <robot_localization/srv/set_datum.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include <string>
#include <math.h>


class datumGen: public rclcpp::Node
{
    public:
        datumGen();
        ~datumGen();
    private:

        //Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;

        //Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;

        //Server clients
        rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr client_datum;

        //Control flags
        bool distance_reached;
        bool first_gps_received;
        bool control_flag;

        //Variables
        geometry_msgs::msg::Twist cmd_vel;
        sensor_msgs::msg::NavSatFix gps1;
        sensor_msgs::msg::NavSatFix gps2;
        uint8_t distance_to_move;

        //Methods
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void datumcallback(rclcpp::Client<robot_localization::srv::SetDatum>::SharedFuture future);
        void send_datum();
};

