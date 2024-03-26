#include "hunter_sim_pkg/String2ros.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;


String2ros::String2ros(): Node("string2ros")
{
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/global",10,std::bind(&String2ros::odomCallback,this,_1));

    sub_pos = this->create_subscription<std_msgs::msg::String>(
            "/web/desired_pos",10,std::bind(&String2ros::posCallback,this,_1));
    sub_gps_pos = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/filtered", 10, std::bind(&String2ros::gpsCallback, this, _1));

    pub_goal = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mqtt/goal", 10);
    pub_goal_array = this->create_publisher<geometry_msgs::msg::PoseArray>("/mqtt/goal_array", 10);
    pub_gt_web = this->create_publisher<std_msgs::msg::String>("/web/gt", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(3000), std::bind(&String2ros::publishMessage, this));

    client_ll = this->create_client<robot_localization::srv::FromLL>("fromLL");

}

String2ros::~String2ros(){}

void String2ros::publishMessage()
{
    
    if(abs(cord.lat - cord_past.lat) > 0.00001 || abs(cord.lng - cord_past.lng) > 0.00001)
    {
    pub_gt_web->publish(json);
    cord_past = cord;
    RCLCPP_INFO(this->get_logger(), "%s",json.data.c_str());
    }

}
void String2ros::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{  
    robot_pose.pose = msg->pose.pose;
    robot_pose.header = msg->header;

}

void String2ros::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    Json::Value root;
    root["lat"] = msg->latitude;
    root["lng"] = msg->longitude;
    cord.lat =  msg->latitude;
    cord.lng =  msg->longitude;

    // Convertir el objeto Json::Value a una cadena JSON
    Json::StreamWriterBuilder writer;
    std::string jsonString = Json::writeString(writer, root);
    json.data = jsonString;

}

void String2ros::posCallback(const std_msgs::msg::String::SharedPtr msg)
{  
    Json::Reader reader;
    Json::Value obj;
    reader.parse(msg->data,obj);
    const std::string tipo = obj["type"].asString();
    RCLCPP_DEBUG(this->get_logger(), "data captured in /web/desired/pos: %s",tipo.c_str());
    if(tipo == "marker")
    {
        marker.lat = obj["lat"].asDouble();
        marker.lng = obj["lng"].asDouble();
        RCLCPP_DEBUG(this->get_logger(), "[lat: %.4f, lng: %.4f]",marker.lat,marker.lng);
        send_request(marker);
        double dx = map_point.x - robot_pose.pose.position.x;
        double dy = map_point.y - robot_pose.pose.position.y;
        double d = sqrt((dx*dx) + (dy*dy));
        if(d < 8)
        {
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position = map_point;
            pub_goal->publish(goal_pose);
        }else{
            int steps = d/8;
            float angle = atan2(dy,dx);
            for(int i = 0;i< steps;i++)
            {
                goal_pose_array.header.frame_id = "map";
                goal_pose_array.header.stamp = this->now();

                goal_pose.pose.position.x = robot_pose.pose.position.x + (i+1)*8*cos(angle);
                goal_pose.pose.position.y = robot_pose.pose.position.y + (i+1)*8*sin(angle);
                goal_pose.pose.position.z = 0;
                RCLCPP_INFO(this->get_logger(), "pos: %.4f, %.4f",goal_pose.pose.position.x,goal_pose.pose.position.y);
                goal_pose_array.poses.push_back(goal_pose.pose);
            }
            goal_pose.pose.position.x = map_point.x;
            goal_pose.pose.position.y = map_point.y;
            goal_pose.pose.position.z = 0;
            goal_pose_array.poses.push_back(goal_pose.pose);

            pub_goal_array->publish(goal_pose_array);

        }


    }else if(tipo == "rectangle")
    {
        rectangle.ne.lat = obj["latne"].asDouble();
        rectangle.ne.lng = obj["lngne"].asDouble();
        rectangle.sw.lat = obj["latsw"].asDouble();
        rectangle.sw.lng = obj["lngsw"].asDouble();
        RCLCPP_DEBUG(this->get_logger(), "[ne:%.4f, %.4f;sw:%.4f,%.4f]",rectangle.ne.lat,rectangle.ne.lng,rectangle.sw.lat,rectangle.sw.lng);

    }else if(tipo == "polygon")
    {
        poly.length = obj["length"].asUInt();
        Json::Value vector = obj["polygon"];
        RCLCPP_DEBUG(this->get_logger(),"%i",poly.length);
        for(u_int8_t i=0;i<poly.length;i++)
        {
            poly.markers[i].lat = vector[i]["lat"].asDouble();
            poly.markers[i].lng = vector[i]["lng"].asDouble();
            RCLCPP_DEBUG(this->get_logger(),"[%.8f, %.8f]",poly.markers[i].lat,poly.markers[i].lng);
        }

    }else if(tipo == "follow")
    {
        marker.lat = obj["lat"].asDouble();
        marker.lng = obj["lng"].asDouble();

        send_request(marker);
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->now();
        goal_pose.pose.position = map_point;
        pub_goal->publish(goal_pose);

    }else{
        RCLCPP_ERROR(this->get_logger(), "Type not defined, or none coordinates in the message");
    }
}

void String2ros::send_request(msg_srv_hunter::msg::Marker marker)
{
    auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
    request->ll_point.latitude = marker.lat;
    request->ll_point.longitude = marker.lng;
    request->ll_point.altitude = 0;

    while (!client_ll->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    RCLCPP_DEBUG(this->get_logger(), "service AVAILABLE, making request....");

    // If everything is okay we Send the async request
    client_ll->async_send_request(request, std::bind(&String2ros::fromllCallback, this, _1));

}

void String2ros::fromllCallback(rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future)
{
    map_point = future.get()->map_point;
}

int main ( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init ( argc, argv );
    
    // Create object from our string2ros class
    auto node = std::make_shared<String2ros>();

    // run at 1Hz
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);    // attend subscriptions and srv request
        loop_rate.sleep();          // sleep till next step time
    }

    rclcpp::shutdown();
    return 0;
}