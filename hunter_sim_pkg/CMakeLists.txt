cmake_minimum_required(VERSION 3.8)
project(hunter_sim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# for services
find_package(rosidl_default_generators REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME} 
  "../robot_localization/srv/SetDatum.srv"
  DEPENDENCIES robot_localization
)


include_directories(
  include
)
#Add executable for Nodes
add_executable(coppelia_simulator src/run_coppelia_simulator.cpp)
add_executable(DatumGen src/datum_gen.cpp)
#Add dependencies for nodes
ament_target_dependencies(coppelia_simulator rclcpp)
ament_target_dependencies(DatumGen rclcpp nav_msgs sensor_msgs tf2_geometry_msgs std_msgs geometry_msgs robot_localization)


target_link_libraries(
  DatumGen
  ${GeographicLib_LIBRARIES}
)


#install directories of pkg
install(DIRECTORY 
  config 
  description 
  launch
  scenes
  mesh
  rviz
  models
  urdf
  DESTINATION share/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(DatumGen "${cpp_typesupport_target}")


#install route to executables
install(TARGETS
  coppelia_simulator
  DatumGen
  DESTINATION lib/${PROJECT_NAME}
)

add_compile_options(-g)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
