#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>


class CoppeliaSim: public rclcpp::Node
{
  public:
    CoppeliaSim()
    : Node("coppelia_simulator")
    {
      //Declare parameters and set default values
      this->declare_parameter<std::string>("coppelia_root_dir", "");
      this->get_parameter("coppelia_root_dir", coppelia_dir);

      this->declare_parameter<std::string>("coppelia_scene_path", "");
      this->get_parameter("coppelia_scene_path", coppelia_scene);
      
      this->declare_parameter<bool>("coppelia_headless", false);
      this->get_parameter("coppelia_headless", coppelia_headless);
      
      // Declare other params (for the scene)
      this->declare_parameter<bool>("show_laser", false);
      this->declare_parameter<bool>("tf_gt", false);
      this->declare_parameter<bool>("tf_odom", true);
      this->declare_parameter<int>("num_beacons", 5);
      this->declare_parameter<float>("var_v",0.0);
      this->declare_parameter<float>("var_w",0.0);
    }
    
    void run()
    {
      //system("~/CoppeliaSim_Edu_V4_3_0_rev10_Ubuntu20_04/coppeliaSim.sh -s ~/tfg_ros_simulation_ws/src/Coppelia_scenes/ros2_mobile_robot.ttt");
      if (coppelia_headless){
        system( (coppelia_dir+"/coppeliaSim.sh -h -s "+ coppelia_scene + " &").c_str() );
      }
      else{
        system( (coppelia_dir+"/coppeliaSim.sh -s "+ coppelia_scene + " &").c_str() );
      }
    }
    
  private:
    std::string coppelia_dir, coppelia_scene;
    bool coppelia_headless;
};

      
// MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  //create object
  std::shared_ptr<CoppeliaSim> myNode = std::make_shared<CoppeliaSim>();

  //launch simulator
  myNode->run();

  // Iterate (for param_server and related services)
  printf("[CoppeliaSimulator] Simulator launched. Now spinning!");
  rclcpp::spin(myNode);

  rclcpp::shutdown();
  return 0;
  
}




