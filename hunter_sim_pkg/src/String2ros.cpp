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

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant );
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,10,std::bind(&String2ros::odomCallback,this,_1));

    sub_pos = this->create_subscription<std_msgs::msg::String>(
            web_goal_topic,10,std::bind(&String2ros::posCallback,this,_1));

    sub_gps_pos = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, 10, std::bind(&String2ros::gpsCallback, this, _1));

    pub_goal = this->create_publisher<geometry_msgs::msg::PoseStamped>(single_goal_topic, 10);
    pub_goal_array = this->create_publisher<geometry_msgs::msg::PoseArray>(multiple_goal_topic, 10);
    client_ll = this->create_client<robot_localization::srv::FromLL>("fromLL",rmw_qos_profile_services_default,client_cb_group_);

    if(send_pose_to_web){
        pub_gt_web = this->create_publisher<std_msgs::msg::String>(publish_web_topic, 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(timer_web), std::bind(&String2ros::publishMessage, this));
    }
}

String2ros::~String2ros(){}


void String2ros::publishMessage()
{
    // this->timer_->cancel();
    pub_gt_web->publish(json);
    if(abs(cord.lat - cord_past.lat) > error || abs(cord.lng - cord_past.lng) > error)
    {
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
        goal_pose_array ={};
        if(d < step)
        {
            goal_pose = {};
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
        RCLCPP_INFO(this->get_logger(), "Received an Area of type RECTANGLE, not working at the moment");
    }else if(tipo == "polygon")
    {
        RCLCPP_INFO(this->get_logger(), "Received an Area of type POLYGON, not working at the moment");
    }else if(tipo == "line"){
        geometry_msgs::msg::PoseArray line;
        poly.length = obj["length"].asUInt();
        Json::Value vector = obj["line"];
        for(u_int8_t i = 0; i < poly.length - 1; i++)
        {
            geometry_msgs::msg::Pose aux;
            poly.markers[i].lat = vector[i]["lat"].asDouble();
            poly.markers[i].lng = vector[i]["lng"].asDouble();

            send_request(poly.markers[i]);
            aux.position = map_point;
            line.poses.push_back(aux);
        }
        double dx = line.poses[0].position.x - robot_pose.pose.position.x;
        double dy = line.poses[0].position.y - robot_pose.pose.position.y;
        double d = sqrt((dx*dx) + (dy*dy));
        goal_pose_array={};
        geometry_msgs::msg::Pose aux;
        if(d < step)
        {
            double angle = atan2(dy,dx);
            tf2::Quaternion q;
            q.setRPY(0,0,angle);
            q.normalize();
            aux.orientation = tf2::toMsg(q);
            aux.position = line.poses[0].position;
            goal_pose_array.poses.push_back(aux);
        }else{
            goal_pose_array = path_generator.get_intermediate(robot_pose.pose.position,line.poses[0].position,step);
        }
        for(u_int8_t i = 0; i < line.poses.size() - 1; i++)
        {
            double dx = line.poses[i+1].position.x - line.poses[i].position.x;
            double dy = line.poses[i+1].position.y - line.poses[i].position.y;
            double d = sqrt((dx*dx) + (dy*dy));
            if(d < step)
            {
                double angle = atan2(dy,dx);
                tf2::Quaternion q;
                q.setRPY(0,0,angle);
                q.normalize();
                aux.orientation = tf2::toMsg(q);
                aux.position = line.poses[i].position;
                goal_pose_array.poses.push_back(aux);
            }else{
                geometry_msgs::msg::PoseArray array_aux;
                array_aux = path_generator.get_intermediate(line.poses[i].position,line.poses[i+1].position,step);
                for(geometry_msgs::msg::Pose p : array_aux.poses)
                {
                    goal_pose_array.poses.push_back(p);
                }
            }
        }
        
        goal_pose_array.header.stamp = robot_pose.header.stamp;
        pub_goal_array->publish(goal_pose_array);

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
    auto response = client_ll->async_send_request(request);
  
    if (response.wait_for(3s) != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive service response");
            return;
        }

    map_point = response.get()->map_point;
    RCLCPP_INFO(this->get_logger(), "[%f, %f] hola",map_point.x,map_point.y);

}


int main ( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init ( argc, argv );
    
    // Create object from our string2ros class
    auto node = std::make_shared<String2ros>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // run at 1Hz
    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok())
    {
        executor.spin();    // attend subscriptions and srv request
        loop_rate.sleep();          // sleep till next step time
    }

    rclcpp::shutdown();
    return 0;
}