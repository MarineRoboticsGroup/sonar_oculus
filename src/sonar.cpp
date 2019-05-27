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

// https://beej.us/guide/bgnet/html/multi/inet_ntoaman.html

#include <arpa/inet.h>
#include <iostream>
#include <sys/ioctl.h>
#include <sys/socket.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

#include "OculusSonar.h"

int main(int argc, char **argv) {

  // Initialize ROS
  ROS_INFO("Initializing...");
  ros::init(argc, argv, "sonar");
  ros::NodeHandle nh("~");
  ros::Publisher ping_pub; // publisher for sonar ping message
  ping_pub = nh.advertise<sonar_oculus::OculusPing>("ping", 1);
  //img_pub = nh.advertise<sensor_msgs::Image>("image", 1); // uncomment to bypass the python viewer (oculus_viewer.py)
 
  // Initialize Sonar
  OculusSonar sonar;
  sonar.fetch_ROS_parameters(nh);
  sonar.connect_to_oculus();

  // Send Ping and initiate data collection
  sonar.fire_oculus();

  ROS_INFO("Entering publishing loop!");

  // run continously
  ros::Rate r(sonar.get_rate_hz()); // pvt: sonar should be under 40Hz (reduced 100 to 50)
  while (ros::ok()) {

    if (sonar.read_from_oculus() > 0) {
        sensor_msgs::Image sonar_image = sonar.get_image();        // image msg
        sonar_oculus::OculusFire fire_msg = sonar.get_fire_msg();  // fire msg
        sonar_oculus::OculusPing ping_msg = sonar.get_ping_msg(sonar_image, fire_msg);  // sonar ping

        // img_pub.publish(sonar_image); // uncomment to bypass the python viewer (oculus_viewer.py)
        ping_pub.publish(ping_msg);
    } 

    sonar.fire_oculus(); // fire sonar (so we sleep while the ping travels)
    r.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Disconnecting...");
  sonar.disconnect_from_oculus();

  return 0;

} // main
