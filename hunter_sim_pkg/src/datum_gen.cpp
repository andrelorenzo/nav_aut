#include "hunter_sim_pkg/datum_gen.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/quaternion.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

datumGen::datumGen(): Node("DatumGen")
{
  //Custom quality of service
  auto qos = rclcpp::SensorDataQoS().reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  //Declaration of parameters
  this->declare_parameter("odom_topic","/odom");
  this->declare_parameter("gps_topic","/gps/fix");
  this->declare_parameter("cmd_vel_topic","/cmd_vel");
  this->declare_parameter("global_imu_topic","/global_imu");
  this->declare_parameter("madgwick_imu_topic","/madgwick_imu");
  this->declare_parameter("distance_to_move",1);
  this->declare_parameter("speed",0.5);
  this->declare_parameter("publish_gloab_imu",false);

  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string gps_topic = this->get_parameter("gps_topic").as_string();
  const std::string vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  const std::string global_imu_topic = this->get_parameter("global_imu_topic").as_string();
  const std::string madgwick_imu_topic = this->get_parameter("madgwick_imu_topic").as_string();
  const int distance2move = this->get_parameter("distance_to_move").as_int();
  const double speed = this->get_parameter("speed").as_double();
  const bool publish_imu = this->get_parameter("publish_gloab_imu").as_bool();

  //Subscribers
      //datumGen
  sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic,qos,std::bind(&datumGen::odomCallback,this,_1));
  sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          gps_topic,qos,std::bind(&datumGen::gpsCallback,this,_1));
 
  //Publishers
      //datumGen
  pub_vel = this->create_publisher<geometry_msgs::msg::Twist>(vel_topic, qos);

  //Client Servers
  client_datum = this->create_client<robot_localization::srv::SetDatum>("datum");


  //Global correction
  if(publish_imu){
  sub_madgwick_imu = this->create_subscription<sensor_msgs::msg::Imu>(
          madgwick_imu_topic, qos, std::bind(&datumGen::madgwickImuCallback, this, _1));
  pub_global_imu = this->create_publisher<sensor_msgs::msg::Imu>(global_imu_topic,qos);
  }

  //Variable initialization
  distance_to_move = distance2move;
  vel = speed;
  first_gps_received = false;
  distance_reached = false;
  control_flag = true;
  cmd_vel = {};
  datum_sended = false;
  imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,
                                          0.0,0.0,0.0,
                                          0.0,0.0,0.0};
  imu_msg.linear_acceleration_covariance = {0.0,0.0,0.0,
                                              0.0,0.0,0.0,
                                              0.0,0.0,0.0};
}

datumGen::~datumGen(){

}

void datumGen::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(first_gps_received && !distance_reached)
  {
    //When the first GPS is received, we start moving forward
    cmd_vel.linear.x = vel;
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

void datumGen::madgwickImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if(datum_sended)
  {
    //  Header with the same frame and the actual time
    imu_msg.header.frame_id = msg->header.frame_id;
    imu_msg.header.stamp = this->now();

    // We set the other fields to the same value in acceleration and velocity
    imu_msg.angular_velocity.x = msg->angular_velocity.x;
    imu_msg.angular_velocity.y = msg->angular_velocity.y;
    imu_msg.angular_velocity.z = msg->angular_velocity.z;

    imu_msg.linear_acceleration.x = msg->linear_acceleration.x;
    imu_msg.linear_acceleration.y = msg->linear_acceleration.y;
    imu_msg.linear_acceleration.z = msg->linear_acceleration.z;

    // For now we create the covariance matrix as a constant
    imu_msg.orientation_covariance = {0.1,0.0,0.0,
                                      0.0,0.1,0.0,
                                      0.0,0.0,0.1};

    imu_msg.angular_velocity_covariance[0] = msg->angular_velocity_covariance[0];
    imu_msg.angular_velocity_covariance[4] = msg->angular_velocity_covariance[4];
    imu_msg.angular_velocity_covariance[8] = msg->angular_velocity_covariance[8];
    
    imu_msg.linear_acceleration_covariance[0] = msg->linear_acceleration_covariance[0];
    imu_msg.linear_acceleration_covariance[4] = msg->linear_acceleration_covariance[4];
    imu_msg.linear_acceleration_covariance[8] = msg->linear_acceleration_covariance[8];


    // To create a global IMU, we add de value of de Datum (aka. Initial global orientation + the local orientation)
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = Datum.orientation.x + msg->orientation.x;
    q_msg.y = Datum.orientation.y + msg->orientation.y;
    q_msg.z = Datum.orientation.z + msg->orientation.z;
    q_msg.w = Datum.orientation.w + msg->orientation.w; 
    tf2::Quaternion q;
    tf2::fromMsg(q_msg,q);
    q.normalize();
    imu_msg.orientation = tf2::toMsg(q);    

    pub_global_imu->publish(imu_msg);
  }
}

void datumGen::send_datum()
{
  //Get the laitudes and longitudes for computing the angle
  double lat1,lat2 = 0;
  lat1 = gps1.latitude * (M_PI/180);
  lat2 = gps2.latitude * (M_PI/180);
  double dl = (gps2.longitude - gps1.longitude) * (M_PI/180);

  //transformation to a plane
  double Y = cos(lat2) * sin(dl);
  double X = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dl);

  // Angle between [-pi, pi]
  double theta = atan2(Y,X);

  // Bearing 0º is East and goes anticlockwise in [0, 2pi], 360 - (angle + 90) == 270 - angle
  double bearing = (2* M_PI/3) - fmod((theta + 2*M_PI),(2*M_PI));
  
  //transform to quaternion
  tf2::Quaternion q;
  q.setRPY(0,0,bearing);
  q.normalize();

  RCLCPP_INFO_STREAM(this->get_logger(), "Theta [-180,180]º:"<<theta*(180/M_PI)<<", Bearing 0º->E [0, 360]º:"<<bearing*(180/M_PI));

  // Get msg to send
  RCLCPP_INFO(this->get_logger(), "Sending: POSITION = [%.2f, %.2f, 0.0] ORIENTATION = [%.2f, %.2f, %.2f, %.2f]",
      gps2.latitude,gps2.longitude,q.x(),q.y(),q.z(),q.w());
  
  // Save the Datum
  Datum.position.latitude = gps2.latitude;
  Datum.position.longitude = gps2.longitude;
  Datum.position.altitude = 0.0;
  Datum.orientation = tf2::toMsg(q);
  //Make the request to the server
  auto request = std::make_shared<robot_localization::srv::SetDatum::Request>();
    request->geo_pose = Datum;

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
  datum_sended = true;
}

int main ( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init ( argc, argv );
    
    // Create object from our datumGen class
    auto node = std::make_shared<datumGen>();

    // run at 1Hz
    rclcpp::Rate loop_rate(5);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);    // attend subscriptions and srv request
        loop_rate.sleep();          // sleep till next step time
    }

    rclcpp::shutdown();
    return 0;
}
