#include "hunter_sim_pkg/String2ros.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;


String2ros::String2ros(): Node("string2ros")
{
    this->declare_parameter("step_area", 8.0);
    step = get_parameter("step_area").as_double();
    this->declare_parameter("odom_topic","/odometry/local");
    const std::string odom_topic = this->get_parameter("odom_topic").as_string();
    this->declare_parameter("web_goal_topic","/web/desired_pos");
    const std::string web_goal_topic = this->get_parameter("web_goal_topic").as_string();
    this->declare_parameter("gps_topic","/gps/fix");
    const std::string gps_topic = this->get_parameter("gps_topic").as_string();
    this->declare_parameter("single_goal_topic","/mqtt/goal");
    const std::string single_goal_topic = this->get_parameter("single_goal_topic").as_string();
    this->declare_parameter("multiple_goals_topic","/mqtt/goal_array");
    const std::string multiple_goal_topic = this->get_parameter("multiple_goals_topic").as_string();
    this->declare_parameter("publish_to_web_pose","/web/gt");
    const std::string publish_web_topic = this->get_parameter("publish_to_web_pose").as_string();
    this->declare_parameter("timer_to_web_send",5);
    const int timer_web = this->get_parameter("timer_to_web_send").as_int() * 1000;
    this->declare_parameter("send_pose_to_web",false);
    const bool send_pose_to_web = this->get_parameter("send_pose_to_web").as_bool();
    this->declare_parameter("error_to_send_actualizaton",0.00001);
    const double error = this->get_parameter("error_to_send_actualizaton").as_double();


    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,10,std::bind(&String2ros::odomCallback,this,_1));

    sub_pos = this->create_subscription<std_msgs::msg::String>(
            web_goal_topic,10,std::bind(&String2ros::posCallback,this,_1));

    sub_gps_pos = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, 10, std::bind(&String2ros::gpsCallback, this, _1));

    pub_goal = this->create_publisher<geometry_msgs::msg::PoseStamped>(single_goal_topic, 10);
    pub_goal_array = this->create_publisher<geometry_msgs::msg::PoseArray>(multiple_goal_topic, 10);
    client_ll = this->create_client<robot_localization::srv::FromLL>("fromLL");

    if(send_pose_to_web){
        pub_gt_web = this->create_publisher<std_msgs::msg::String>(publish_web_topic, 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(timer_web), std::bind(&String2ros::publishMessage, this));
    }
}

String2ros::~String2ros(){}


void String2ros::publishMessage()
{
    this->timer_->cancel();
    if(abs(cord.lat - cord_past.lat) > error || abs(cord.lng - cord_past.lng) > error)
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
    if(tipo == "marker" || tipo =="follow")
    {

        marker.lat = obj["lat"].asDouble();
        marker.lng = obj["lng"].asDouble();
        RCLCPP_DEBUG(this->get_logger(), "[lat: %.4f, lng: %.4f]",marker.lat,marker.lng);
        send_request(marker);
        double dx = map_point.x - robot_pose.pose.position.x;
        double dy = map_point.y - robot_pose.pose.position.y;
        double d = sqrt((dx*dx) + (dy*dy));
        goal_pose_array={};
        if(d < step)
        {
            double angle = atan2(dy,dx);
            tf2::Quaternion q;
            q.setRPY(0,0,(angle));
            q.normalize();
            goal_pose.pose.orientation = tf2::toMsg(q); 
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position = map_point;
            pub_goal->publish(goal_pose);
        }else{
            goal_pose_array = path_generator.get_intermediate(robot_pose.pose.position,map_point,step);
            goal_pose_array.header.stamp = robot_pose.header.stamp;
            pub_goal_array->publish(goal_pose_array);
        }

    }else if(tipo == "rectangle")
    {
        rectangle.ne.lat = obj["latne"].asDouble();
        rectangle.ne.lng = obj["lngne"].asDouble();
        rectangle.sw.lat = obj["latsw"].asDouble();
        rectangle.sw.lng = obj["lngsw"].asDouble();
        geometry_msgs::msg::PoseArray poligon;

        send_request(rectangle.ne);
        geometry_msgs::msg::Pose aux_ne;
        aux_ne.position = map_point;

        send_request(rectangle.sw);
        geometry_msgs::msg::Pose aux_sw;
        aux_sw.position = map_point;

        geometry_msgs::msg::Pose aux_nw;
        aux_nw.position.x = aux_sw.position.x;
        aux_nw.position.y = aux_ne.position.y;

        geometry_msgs::msg::Pose aux_se;
        aux_se.position.x = aux_ne.position.x;
        aux_se.position.y = aux_sw.position.y;
        poligon.poses.push_back(aux_nw);
        poligon.poses.push_back(aux_ne);
        poligon.poses.push_back(aux_se);
        poligon.poses.push_back(aux_sw);

        geometry_msgs::msg::PoseArray path = path_generator.get_area_path(poligon,step);
        path.header.stamp = robot_pose.header.stamp;
        path.header.frame_id = "map";
        pub_goal_array->publish(path);

        RCLCPP_DEBUG(this->get_logger(), "[ne:%.4f, %.4f;sw:%.4f,%.4f]",rectangle.ne.lat,rectangle.ne.lng,rectangle.sw.lat,rectangle.sw.lng);

    }else if(tipo == "polygon")
    {
        geometry_msgs::msg::PoseArray poligon;
        poly.length = obj["length"].asUInt();
        Json::Value vector = obj["polygon"];
        RCLCPP_DEBUG(this->get_logger(),"%i",poly.length);

        for(u_int8_t i=0;i<poly.length;i++)
        {
            geometry_msgs::msg::Pose aux;
            poly.markers[i].lat = vector[i]["lat"].asDouble();
            poly.markers[i].lng = vector[i]["lng"].asDouble();
            send_request(poly.markers[i]);
            aux.position = map_point;
            poligon.poses.push_back(aux);
            RCLCPP_DEBUG(this->get_logger(),"[%.8f, %.8f]",poly.markers[i].lat,poly.markers[i].lng);
        }
        geometry_msgs::msg::PoseArray path = path_generator.get_area_path(poligon,step);
        path.header.stamp = robot_pose.header.stamp;
        path.header.frame_id = "map";
        pub_goal_array->publish(path);

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