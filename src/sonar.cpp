/**
 * @file sonar.cpp
 * @author Vincent Sieben
 * @author Pedro Vaz Teixeira (pvt@mit.edu)
 * @author Timothy Osedach (tosedach@slb.com)
 * @brief ROS publisher for Blueprint Oculus multibeam sonars
 * @version 0.1
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

#include "OculusSonar.h"


int main(int argc, char **argv) {
  // Initialize ROS node
  ROS_INFO("Initializing ROS node for Oculus multibeam sonar...");
  ros::init(argc, argv, "sonar");
  ros::NodeHandle nh("~");
  OculusSonar sonar(nh);          // Initialize Sonar

  ROS_INFO("Establishing TCP connection to sonar...");
  sonar.connect_to_oculus();      // Establish TCP connection with sonar

  if (sonar.get_trigger_mode() == 0) {  // asynchronous mode
    sonar.fire_oculus(); // Send Ping and initiate data collection
    ROS_INFO("Entering publishing loop!");
    ros::Rate sample_rate(sonar.get_rate_hz()); 
    ros::Rate poll_rate(POLL_RATE);
    while (ros::ok()) {   // run continously
      while (ros::ok()) {
        if (sonar.check_for_ping() == true)
          break;

        poll_rate.sleep();
        ros::spinOnce();  // tpo: is this needed?
      }

      sonar.fire_oculus();  // fire sonar (so we sleep while the ping travels)
      sample_rate.sleep();
      ros::spinOnce();    // tpo: is this needed?
    }
  }
  else {  // external trigger mode
    ROS_INFO("Ready to fire sonar...");
    ros::spin();
  }

  ROS_INFO("Closing TCP connection to the sonar...");
  sonar.disconnect_from_oculus();
  return 0;
} // main
