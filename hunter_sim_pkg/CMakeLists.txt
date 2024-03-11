cmake_minimum_required(VERSION 3.8)
project(hunter_sim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

#Add executable for Nodes
add_executable(coppelia_simulator src/run_coppelia_simulator)

#Add dependencies for nodes
ament_target_dependencies(coppelia_simulator rclcpp)


#install route to executables
install(TARGETS
  coppelia_simulator
  DESTINATION lib/${PROJECT_NAME}
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
  DESTINATION share/${PROJECT_NAME}
)

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
