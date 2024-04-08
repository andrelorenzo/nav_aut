# ACKERMAN REPO FOR AUTONOMOUS NAVIGATION

This repository has been created by André Lorenzo Torres, for its final degree project, itś been created for an ackerman type robot (960 x 300 mm) with one GPS RTK (Reach RS2+), one Lidar 3D + integrated IMU witout magnetometer(Ouster ...) and a Agilex hunter v2.0 chasis, it's computer is a ..

- #### [Interfaz_Web_Server](Interfaz_web-server)
This folder contains the code and the build for a web server to serve as an interface, uses a google API, itś recommended to copy the build folder and run it in a private server, as I'm writing this my setup for this task is a raspberry pi 3b+ using a nginx server with a simple authetification protocol. This web server has been created using React and a couple of npm libraries.

As mention above uses a google API key to get 4 different types of shapes and sends its coordinates via mqtt, also has a built-in follow-me action, the implemented uses are
-  move the marker to a position and send its coordinates via the "desired_pos" topic.
-  create a rectangle area and send its coordinates via the "desired_pos" topic.
-  create a polygon area and send its coordinates via the "desired_pos" topic.
-  create a polilyne (or path) and send its coordinates via the "desired_pos" topic.
-  activate the follow me button, this will send an activation flag via the "follow" topic and will start to send every 5 seconds the device position via the "desired_pos" topic.
-  subscribes to another topic "position" and adds a marker in the position that it receives.

In any case for testing porpouse you can test it locally with the following code:

1. Install Node Js and npm
```
sudo apt install nodejs
sudo apt install npm
```
2. Go to the proyect folder (interfaz_react) and install dependencies
```
cd /to/the/proyect/folder
npm install
```
3. Start the web server on port 3000
```  
npm start
```

- #### Submodules
As you can see in the folder there is 3 submodules, the first one [*imu_tools*](imu_tools) is used for it's `complementary_filter`, this filter will subscribe to a IMU message with accelerometer, gyroscope and optionally a magnetometer and will output an orientation, also features bias.

The second submodule [*mqtt_client*](mqtt_client) is a simple bridge between MQTT and ROS2 subscribes to the different topics and publishes whenever needed
The third and most important is [*robot_localization*](robot_localization), this package features a standarised way to fuse multiple sources of odometry into 2 `ekf_filters`, for this functionality you can downloaded from source but here i also use its server for transforming gps coordinates into a cartesian representation.
