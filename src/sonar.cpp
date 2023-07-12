/**
 * @file sonar.cpp
 * @brief ROS publisher for Blueprint Oculus multibeam sonars
 * @version 1.0
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

#include <sonar_oculus/OculusSonar.h>

int main(int argc, char **argv) {
  // Initialize ROS node
  ROS_INFO("Initializing ROS node for Oculus multibeam sonar...");
  ros::init(argc, argv, "sonar");
  ros::NodeHandle nh("~");
  OculusSonar sonar(nh);          // Initialize Sonar

  ROS_INFO("Establishing TCP connection to sonar...");
  //sonar.get_oculus_ip_address();      // Establish TCP connection with sonar
  sonar.connect_to_oculus();
  
  if (sonar.get_trigger_mode() == 0) {  // asynchronous mode

    ROS_INFO("Entering publishing loop!");
    ros::Rate sample_rate(sonar.get_rate_hz()); 
    sonar.fire_oculus(); // Send Ping and initiate data collection
    while (ros::ok()) {   // run continously
      if (sonar.process_ping()) {
        // std::cout << "ping detected..." << std::endl;
        sonar.fire_oculus();  
      }

      //std::cout << "firing sonar..." << std::endl;
      //sonar.fire_oculus();  // fire sonar (so we sleep while the ping travels)
     
      sample_rate.sleep();
      ros::spinOnce();   
    }
  }
  else {  // external trigger mode
    ROS_INFO("Ready to fire sonar...");
    sonar.fire_oculus(); // Send Ping and initiate data collection
    ros::spin();
  }

  ROS_INFO("Closing TCP connection to the sonar...");
  sonar.disconnect_from_oculus();
  return 0;
} // main
