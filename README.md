# Differential Drive robot on ESP32 with microROS integration


This is code for a differential drive robot using just an ESP32 as the main brain. This code has microROS integrated into it which means that the robot will have access to the entire ROS ecosystem. The benefits being I can control the robot using packages created by the ROS community without the need of an SBC (or greater). Soon after a encoder and range sensors are integrated, navigation using the ROS framework will be possible. 

## Usage

Please refer to [here](https://github.com/micro-ROS/micro_ros_espidf_component) for files and packages needed for microROS on ESP32 along with how to build, configure the network and flash programs into the microcontroller. Refer [here](https://micro.ros.org/docs/tutorials/core/first_application_linux/) for setting up the microROS agent on your host machine. 

After the program is flashed and the host machine is set up, power up your robot and run the following on your host machine:

```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

After a few seconds, the robot and the host machine should be connected and you may control the robot using a teleoperation node such as the following:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Issues

Some issues I faced was in creating my own IDF project. The microROS ESP32 github I linked above advised to clone the github repo into a components directory within the project and to later run configurations, build and flash. But when it came to building I would repeatedly get an error. I discuss this problem [here](https://github.com/micro-ROS/micro_ros_espidf_component/issues/201). To get past this issue I edited one of the example codes that was in the microROS ESP32 github and replaced that code with my code, rebuilt and flashed. This seemed to work fine.

## Benefits of microROS and ESP32

microROS allows a microcontroller to act as its own independent node and when you combine that with the Wifi capabilities of the ESP32, you find yourself with an independant wireless node without the need of SBCs like Raspberry Pis, Jetson Nanos and/or laptops. This also means that you no longer need to do any master-slave configuration. Finally, this will bring the costs down for robots by a great bit with the negation of these SBCs
