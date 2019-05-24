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
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "OculusClient.h"

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

// Dynamic server
#include <dynamic_reconfigure/server.h>
#include <sonar_oculus/OculusParamsConfig.h>

#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100

// Global sonar configuration
// TODO: move these into a SonarConfig class to avoid global variables
int mode = 1;             // 0 => dev/not used, 1 => ~750khz, 2 => ~1.2Mhz.
double range = 5;        // m, limited to 120m in mode 1, and 40m in mode 2
double gain = 20;         // %
double soundspeed = 1500; // m/s
double salinity = 0;      // ppm, 0 = freshwater, 35=saltwater
int threshold = 90;       // intensity threshold

/**
 * @brief Error handling function
 * 
 * @param msg Error message
 */
void error(const char *msg) {
  perror(msg);
  exit(0);
}

/**
 * @brief Dynamic reconfigure server callback
 * 
 * @param config 
 * @param level 
 */
void callback(sonar_oculus::OculusParamsConfig &config, uint32_t level) {
  mode = config.Mode;
  gain = config.Gain;
  soundspeed = config.Speed;
  range = config.Range;
  salinity = config.Salinity;
  threshold = config.Threshold;

  ROS_INFO("Reconfigure Request: %i %i %i %i %i", config.Mode, config.Gain,
           config.Speed, config.Range, config.Salinity);
}

void get_oculus_ip_address(struct in_addr &ip_address) {
  struct sockaddr_in serverUDP, clientUDP;
  int sockUDP;  
  int buf_size = DATALEN;
  int keepalive = 1;
  socklen_t lengthServerUDP, lengthClientUDP;

  // Clear and initialize values of server and client network info
  lengthServerUDP = sizeof(serverUDP);
  bzero((char *)&serverUDP, lengthServerUDP);
  serverUDP.sin_family = AF_INET;
  serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
  serverUDP.sin_port = htons(PORT_UDP);
  lengthClientUDP = sizeof(clientUDP);

  // Create the UDP listening socket or exit
  sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockUDP < 0)
    error("Error opening UDP listening socket");

  // Bind the UDP socket to address and port, or exit with error
  if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0)
    error("Error binding UDP listening socket");
  listen(sockUDP, 5);

  int64_t bytesAvailable = 0;
  OculusStatusMsg osm;

  while (true) {
    ioctl(sockUDP, FIONREAD, &bytesAvailable);
    if (bytesAvailable > 0) {
      unsigned bytesRead = read(sockUDP, (char *)&osm, bytesAvailable);
      close(sockUDP);
      ip_address.s_addr = osm.ipAddr;
      //printf("The IP address is %s\n", inet_ntoa(ip_address));
      break;
    }
    else {
      // Do Nothing
    }
      
    ROS_INFO(".");
    ros::Duration(1.0).sleep();
  }
}

int get_socket(struct in_addr ip_address) {     /// WHY IS THIS A PROBLEM???  in_addr not highlighting...
  struct sockaddr_in serverTCP; 
  int sockTCP;  
  int buf_size = DATALEN;
  int keepalive = 1;
  socklen_t lengthServerTCP; 

  bzero((char *)&serverTCP, lengthServerTCP);
  serverTCP.sin_family = AF_INET;
  serverTCP.sin_addr.s_addr = ip_address.s_addr;
  serverTCP.sin_port = htons(PORT_TCP);
  lengthServerTCP = sizeof(serverTCP);

  // Create the TCP socket for main communication or exit
  sockTCP = socket(AF_INET, SOCK_STREAM, 0);
  if (sockTCP < 0)
    error("Error opening TCP main socket");

  // Connect to the sonar Server via TCP socket or exit with error
  if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0)
    error("Error connecting TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0)
    error("Error increasing RCVBUF for TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0)
    error("Error keeping alive option set for TCP socket");
  listen(sockTCP, 5);

  return sockTCP;
}


// Main program for listening to sonar
int main(int argc, char **argv) {

  // Initialize ROS
  ROS_INFO("Initializing...");
  ros::init(argc, argv, "sonar");
  ros::NodeHandle nh("~");

  // Sonar info
  unsigned int nbeams = 0, nbins = 0;
  std::string frame_str;
  std::string ip_address_str;
  struct in_addr ip_address;
  unsigned int latest_id = 0; // keep track of latest ping to avoid republishing
  int sockTCP;

  // Fetch needed ROS parameters
  if (nh.getParam("ip_address", ip_address_str)) { 
    inet_aton(ip_address_str.c_str(), &ip_address);
    ROS_INFO("Imported parameter, \"ip_address\": %s", inet_ntoa(ip_address));
  }
  else { 
    ROS_WARN("Failed to import parameter, \"ip_address\".  Will attempt to autodetect sonar...");
    get_oculus_ip_address(ip_address);
    ROS_INFO("Oculus sonar detected at %s.", inet_ntoa(ip_address));
  }

  if (nh.getParam("frame", frame_str)) {
    ROS_INFO("Imported parameter, \"frame\": %s", frame_str.c_str());
  } else {
    ROS_ERROR("Failed to import parameter, \"frame\".");
    frame_str = "sonar";  // default
  }

  ros::Publisher ping_pub; // publisher for sonar ping message
  ping_pub = nh.advertise<sonar_oculus::OculusPing>("ping", 1);
  //img_pub = nh.advertise<sensor_msgs::Image>("image", 1); // uncomment to bypass the python viewer (oculus_viewer.py)

  // Setup dynamic server
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig> serverParam;
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  serverParam.setCallback(f);

  // Create sonar oculus control class
  OsClientCtrl sonar;

  // Setup TCP connection with sonar
  std::cout << "Getting socket!\n";   // magically needed
  sockTCP = get_socket(ip_address);
  //std::cout << "Got socket " << sockTCP << "\n";
  std::cout << "Using socket: " << sockTCP << " for sonar at " << inet_ntoa(ip_address) << "\n";
  sonar.m_readData.m_pSocket = &sockTCP;   // Pass the socket to the control

  sonar.Connect();
  ROS_INFO("Connected to sonar!");

  // Send Ping and initiate data collection
  sonar.Fire(mode, range, gain, soundspeed, salinity);

  ROS_INFO("Entering publishing loop!");

  // run continously
  ros::Rate r(10); // pvt: sonar should be under 40Hz (reduced 100 to 50), tpo: this should be parameterized
  while (ros::ok()) {

    // run the read thread sonar
    sonar.m_readData.run();

    // get bins and beams #.
    nbins = sonar.m_readData.m_osBuffer[0].m_rfm.nRanges;
    nbeams = sonar.m_readData.m_osBuffer[0].m_rfm.nBeams;
    unsigned int id = sonar.m_readData.m_osBuffer[0].m_rfm.pingId;

    // create pointcloud message from sonar data
    if (nbeams > 0 && nbins > 0 && id > latest_id) {
      latest_id = id;
      //ROS_INFO("Latest ID: %i", latest_id);
      // sonar image
      if (sonar.m_readData.m_osBuffer[0].m_rawSize) {
        sensor_msgs::Image sonar_image;
        sonar_image.header.stamp = ros::Time::now();
        sonar_image.height = nbins;
        sonar_image.width = nbeams;
        sonar_image.encoding = "8UC1";
        // sonar_image.is_bigendian = 0; // default works
        sonar_image.step = nbins;
        sonar_image.data.resize(nbeams * nbins);
        std::copy(sonar.m_readData.m_osBuffer[0].m_pImage,
                  sonar.m_readData.m_osBuffer[0].m_pImage + nbins * nbeams,
                  sonar_image.data.begin());
        // img_pub.publish(sonar_image); // uncomment to bypass the python viewer (oculus_viewer.py)

        // fire msg
        sonar_oculus::OculusFire fire_msg;
        fire_msg.header.stamp = ros::Time::now();

        fire_msg.mode = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
        fire_msg.gamma = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
        fire_msg.flags = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
        fire_msg.range = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
        fire_msg.gain = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
        fire_msg.speed_of_sound = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
        fire_msg.salinity = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;

        // sonar ping
        sonar_oculus::OculusPing ping_msg;
        ping_msg.header.frame_id = frame_str;
        ping_msg.header.stamp = fire_msg.header.stamp;
        ping_msg.ping = sonar_image;
        ping_msg.fire_msg = fire_msg;
        ping_msg.ping_id = id;
        ping_msg.status = sonar.m_readData.m_osBuffer[0].m_rfm.status;
        ping_msg.frequency = sonar.m_readData.m_osBuffer[0].m_rfm.frequency;
        ping_msg.temperature = sonar.m_readData.m_osBuffer[0].m_rfm.temperature;
        ping_msg.pressure = sonar.m_readData.m_osBuffer[0].m_rfm.pressure;
        ping_msg.speed_of_sound = sonar.m_readData.m_osBuffer[0].m_rfm.speedOfSoundUsed;

        ping_msg.start_time = sonar.m_readData.m_osBuffer[0].m_rfm.pingStartTime;
        ping_msg.bearings.resize(nbeams);
        for (unsigned int i = 0; i < nbeams; ++i)
          ping_msg.bearings[i] = sonar.m_readData.m_osBuffer[0].m_pBrgs[i];
        ping_msg.range_resolution = sonar.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
        ping_msg.num_ranges = nbins;
        ping_msg.num_beams = nbeams;

        ping_pub.publish(ping_msg);
      }

    } // if (nbins>0 && nbeams>0 && id>latest_id)

    // fire sonar (so we sleep while the ping travels)
    sonar.Fire(mode, range, gain, soundspeed, salinity);
    r.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Disconnecting...");
  sonar.Disconnect();

  // close sockets
  close(sockTCP);
  return 0;

} // main
