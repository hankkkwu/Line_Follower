/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file rikirobot.cpp
*@author Sudarshan Raghunathan
*@brief  Functions definitions for rikirobot class
*/

#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "rikirobot.hpp"
#include "riki_line_follower/pos.h"
#include "linedetect.hpp"

void rikirobot::dir_sub(riki_line_follower::pos msg) {
    rikirobot::dir = msg.direction;
}

void rikirobot::cte_sub(riki_line_follower::cte cte_msg) {
    rikirobot::cte = cte_msg.cross_track_error;
}

void rikirobot::vel_cmd(geometry_msgs::Twist &velocity, ros::Publisher &pub, ros::Rate &rate, PID &pid) {
    // If direction = 3, searching for red line
    if (rikirobot::dir == 3) {
        velocity.linear.x = 0;
        velocity.angular.z = 0.15;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
    }
    else {
        velocity.linear.x = 0.4;

        // cte is an extern value from linedetect.cpp
        pid.UpdateError(rikirobot::cte);
        double angular_velocity = pid.TotalError();
	if (angular_velocity > 1.5){
	    angular_velocity = 1.5;
	}
	else if (angular_velocity < -1.5){
	    angular_velocity = -1.5;
	}

        velocity.angular.z = angular_velocity;
        pub.publish(velocity);
        rate.sleep();

        ROS_INFO("cte: %f,   angular_velocity: %f", rikirobot::cte, angular_velocity);

        if (angular_velocity > 0.02){
            ROS_INFO_STREAM("Turning Left");
        }
        else if (angular_velocity < -0.02){
            ROS_INFO_STREAM("Turning Right");
        }
        else{
            ROS_INFO_STREAM("Going Straight");
        }
    }
}

/*
void rikirobot::vel_cmd(geometry_msgs::Twist &velocity,
 ros::Publisher &pub, ros::Rate &rate) {
    // If direction is left
    if (rikirobot::dir == 0) {
        velocity.linear.x = 0.05;
        velocity.angular.z = 0.05;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Left");
    }
    // If direction is straight
    if (rikirobot::dir == 1) {
        velocity.linear.x = 0.05;
        velocity.angular.z = 0;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Straight");
    }
    // If direction is right
    if (rikirobot::dir == 2) {
        velocity.linear.x = 0.05;
        velocity.angular.z = -0.05;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Right");
    }
    // If robot has to search
    if (rikirobot::dir == 3) {
        velocity.linear.x = 0;
        velocity.angular.z = 0.15;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
    }
}
*/
