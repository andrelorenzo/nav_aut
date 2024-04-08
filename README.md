# ACKERMAN REPO FOR AUTONOMOUS NAVIGATION

This repository has been created by André Lorenzo Torres, for its final degree project, itś been created for an ackerman type robot (960 x 300 mm) with one GPS RTK (Reach RS2+), one Lidar 3D + integrated IMU witout magnetometer(Ouster ...) and a Agilex hunter v2.0 chasis, it's computer is a ..

- #### [Interfaz_Web_Server](Interfaz_web-server)
This folder contains the code and the build for a web server to serve as a interface, uses a google API, itś recommended to copy the build folder and run it in a private server, as I'm writing this my setup for this task is a raspberry pi 3b+ using a nginx server with a simple authetification protocol. This web server has been created using React and a couple of npm libraries 

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
