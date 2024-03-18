#include "hunter_sim_pkg/datum_gen.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

datumGen::datumGen(): Node("DatumGen")
{
  //Custom quality of service
  auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(10));

  //Declaration of parameters
  this->declare_parameter("odom_topic","/odom");
  this->declare_parameter("gps_topic","/gps/fix");
  this->declare_parameter("cmd_vel_topic","/cmd_vel");
  this->declare_parameter("distance_to_move",1);
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string gps_topic = this->get_parameter("gps_topic").as_string();
  const std::string vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  const int distance2move= this->get_parameter("distance_to_move").as_int();

  //Subscribers
  sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic,qos,std::bind(&datumGen::odomCallback,this,_1));
  sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          gps_topic,qos,std::bind(&datumGen::gpsCallback,this,_1));

  //Publishers
  pub_vel = this->create_publisher<geometry_msgs::msg::Twist>(vel_topic, qos);
  
  //Client Servers
  client_datum = this->create_client<robot_localization::srv::SetDatum>("datum");

  //Variable initialization
  distance_to_move = distance2move;
  first_gps_received = false;
  distance_reached = false;
  control_flag = true;
  cmd_vel = {};

}

datumGen::~datumGen(){

}

void datumGen::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(first_gps_received && !distance_reached)
  {
    //When the first GPS is received, we start moving forward
    cmd_vel.linear.x = 0.5;
    RCLCPP_INFO(this->get_logger(), "Moving forward, %.2f meters left......[%.2f m/s]",(1-(msg->pose.pose.position.x)),cmd_vel.linear.x);
  }else
  {
    cmd_vel.linear.x = 0;
  }
  pub_vel->publish(cmd_vel);
  if(msg->pose.pose.position.x >= distance_to_move)
  {
    //When the required distance is reached, we are able to get the second GPS coordinate
    RCLCPP_DEBUG(this->get_logger(), "%.2i meter(s) reached, shutting down subscriber/publisher and getting second GPS",distance_to_move);
    distance_reached = true;
  }
}

void datumGen::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!first_gps_received || distance_reached)
  {
    first_gps_received = true;
    if(first_gps_received && control_flag)
    {
      //When the first GPS coordinate appears we save it
      gps1.latitude = msg->latitude;
      gps1.longitude = msg->longitude;
      RCLCPP_INFO_STREAM(this->get_logger(),"First GPS adquire [lat:" << gps1.latitude <<", lng:"<< gps1.longitude<<"]");
      control_flag = false;
    }else
    {
      //When the robot_reaches the required distance we keep the second GPS coordinate, shutdown every sub/pub adn prepare de datum sender
      gps2.latitude = msg->latitude;
      gps2.longitude = msg->longitude;
      RCLCPP_INFO_STREAM(this->get_logger(),"Second GPS adquire [lat:" << gps2.latitude <<", lng:"<< gps2.longitude<<"]");
      RCLCPP_DEBUG(this->get_logger(), "Setting up datum...%.2f",(gps2.latitude - gps1.latitude));
      pub_vel.reset();
      sub_odom.reset();
      sub_gps.reset();
      send_datum();
    }
  }
}

void datumGen::send_datum()
{
  //Get the laitudes and longitudes for computing the angle
  double lat1,lat2 = 0;
  lat1 = gps1.latitude * (M_PI/180);
  lat2 = gps2.latitude * (M_PI/180);
  double dl = abs(gps2.longitude - gps1.longitude) * (M_PI/180);

  //transformation to a plane
  double Y = cos(lat2) * sin(dl);
  double X = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dl);

  // Angle between [-pi, pi]
  //We substract pi/2 because we want 0º on East, normally Bearing is 0º on North
  double theta = atan2(Y,X) - (M_PI/2);

  // Bearing 0º is East and goes clockwise in [0, 2pi]
  double bearing = fmod((theta + 2*M_PI),(2*M_PI));
  
  //transform to quaternion
  tf2::Quaternion q;
  q.setRPY(0,0,bearing);
  q.normalize();

  RCLCPP_INFO_STREAM(this->get_logger(), "Theta [-90,90]:"<<theta*(180/M_PI)<<", Bearing 0º=>E [0, 360]:"<<bearing*(180/M_PI));

  // Get msg to send
  RCLCPP_INFO(this->get_logger(), "Sending: POSITION = [%.2f, %.2f, 0.0] ORIENTATION = [%.2f, %.2f, %.2f, %.2f]",
      gps2.latitude,gps2.longitude,q.x(),q.y(),q.z(),q.w());

  //Make the request to the server
  auto request = std::make_shared<robot_localization::srv::SetDatum::Request>();
    request->geo_pose.position.latitude = gps2.latitude;
    request->geo_pose.position.longitude = gps2.longitude;
    request->geo_pose.position.altitude = 0.0;
    request->geo_pose.orientation = tf2::toMsg(q);

  while (!client_datum->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_DEBUG(this->get_logger(), "service AVAILABLE, making request....");

  // If everything okay we Send the async request
  client_datum->async_send_request(request, std::bind(&datumGen::datumcallback, this, _1));

}
// MAIN //

void datumGen::datumcallback(rclcpp::Client<robot_localization::srv::SetDatum>::SharedFuture future)
{
  //Just to make sure everything went smoothly..
  RCLCPP_INFO(this->get_logger(), "SUCCES!!, datum sended... ");
}

int main ( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init ( argc, argv );
    
    // Create object from our datumGen class
    auto node = std::make_shared<datumGen>();

    // run at 1Hz
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);    // attend subscriptions and srv request
        loop_rate.sleep();          // sleep till next step time
    }

    rclcpp::shutdown();
    return 0;
}
