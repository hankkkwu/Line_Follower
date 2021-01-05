# Line Follower Rikirobot

The goal of this project is to make a robot to move along the red line.

### Files
This project includes the following files:
- "detect.cpp" contains detection node, which subscribes to image topic and publish the cross track error and direction messages.
- "linedetect.cpp" contains a image processor, which will find the red line and calculate the cross track error for PID controller.
- "rikirobot.cpp" contains a velocity publisher, which will publish the linear and angular velocity to cmd_vel topic.
- "motion_node.cpp" contains a Velocity node, which will initialize the PID parameters.
- "pid.cpp" contains a PID controller, that will calculate the angular velocity.


### Environment
The robot car environment:
- Jetson Nano
- STM32F103RCT6 motor controller
- Ubuntu 18.04
- ROS Melodic
