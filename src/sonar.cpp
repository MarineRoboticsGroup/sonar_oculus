/**
 * @file sonar.cpp
 * @author Vincent Sieben (VSieben@slb.com)
 * @author Pedro Vaz Teixeira (pvt@mit.edu)
 * @brief ROS publisher for Blueprint Oculus multibeam sonars
 * @version 0.1
 * 
 * @copyright Copyright (c) 2019
 * 
 */
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

// #define BUFLEN 200
#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100
//#define SONAR_ADDR "169.254.37.89"
#define SONAR_ADDR "192.168.2.4" // TODO: make this a command line argument
#define PI 3.14159265359

// Global sonar configuration
// TODO: move these into a SonarConfig class to avoid global variables
int mode = 1;             // 0 => dev/not used, 1 => ~750khz, 2 => ~1.2Mhz.
double range = 10;        // m, limited to 120m in mode 1, and 40m in mode 2
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

// Main program for listening to sonar
int main(int argc, char **argv) {
  // Initialize ROS
  ROS_INFO("Initializing...");
  ros::init(argc, argv, "sonar_oculus");
  ros::NodeHandle nh("~");

  unsigned int latest_id = 0; // keep track of latest ping to avoid republishing

  ros::Publisher img_pub, ping_pub;
  img_pub = nh.advertise<sensor_msgs::Image>("image", 1);
  ping_pub = nh.advertise<sonar_oculus::OculusPing>("ping", 1);

  // Setup dynamic server
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig> serverParam;
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  serverParam.setCallback(f);

  // Communications
  struct sockaddr_in serverUDP, clientUDP;
  struct sockaddr_in serverTCP; //, clientTCP;
  int sockUDP, sockTCP;         // sockTCPfd, datagramSize, n;
  int buf_size = DATALEN;
  int keepalive = 1;
  socklen_t lengthServerUDP, lengthClientUDP;
  socklen_t lengthServerTCP; //, lengthClientTCP;
  // char datagramMessage[BUFLEN], buffer[BUFLEN], sonardata[DATALEN];

  // Create sonar oculus control class
  OsClientCtrl m750d;

  // Sonar info
  unsigned int nbeams = 0, nbins = 0;
  std::string frame_str;

  // Clear and intialize values of server and client network info
  lengthServerUDP = sizeof(serverUDP);
  bzero((char *)&serverUDP, lengthServerUDP);
  serverUDP.sin_family = AF_INET;
  serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
  // serverUDP.sin_addr.s_addr = inet_addr(SONAR_ADDR);
  serverUDP.sin_port = htons(PORT_UDP);

  lengthClientUDP = sizeof(clientUDP);
  lengthServerTCP = sizeof(serverTCP);

  // Create the UDP listening socket or exit
  sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockUDP < 0)
    error("Error opening UDP listening socket");

  // Bind the UDP socket to address and port, or exit with error
  if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0)
    error("Error binding UDP listening socket");
  listen(sockUDP, 5);

  ROS_INFO("Connecting");
  while (true) {
    int64_t bytesAvailable;
    ioctl(sockUDP, FIONREAD, &bytesAvailable);

    OculusStatusMsg osm;
    if (bytesAvailable > 0) {
      unsigned bytesRead = read(sockUDP, (char *)&osm, bytesAvailable);
      struct in_addr ip_addr;
      ip_addr.s_addr = osm.ipAddr;
      printf("The IP address is %s\n", inet_ntoa(ip_addr));

      bzero((char *)&serverTCP, lengthServerTCP);
      serverTCP.sin_family = AF_INET;
      serverTCP.sin_addr.s_addr = osm.ipAddr;
      serverTCP.sin_port = htons(PORT_TCP);

      // Create the TCP socket for main communication or exit
      sockTCP = socket(AF_INET, SOCK_STREAM, 0);
      if (sockTCP < 0)
        error("Error opening TCP main socket");
      // Connect to the sonar Server via TCP socket or exit with error
      if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0)
        error("Error connecting TCP socket");
      if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size,
                     sizeof(buf_size)) < 0)
        error("Error increasing RCVBUF for TCP socket");
      if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive,
                     sizeof(keepalive)) < 0)
        error("Error keeping alive option set for TCP socket");
      listen(sockTCP, 5);
      break;
    }
    ROS_INFO(".");
    ros::Duration(1.0).sleep();
  }

  // Setup Sonar and messages
  // Pass the socket to the control
  m750d.m_readData.m_pSocket = &sockTCP;

  m750d.Connect();

  ROS_INFO("Connected!");

  // Send Ping and initiate data collection
  m750d.Fire(mode, range, gain, soundspeed, salinity);
  // Get frame ID
  if (nh.getParam("frame", frame_str)) {
    ROS_INFO("Got param: %s", frame_str.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frame'");
    frame_str = "sonar";
  }
  // sonar_cloud.header.frame_id = frame_str.c_str();
  // sonar_laserEQ.header.frame_id = frame_str.c_str();

  ROS_INFO("Entering publishing loop!");

  // Run continously
  ros::Rate r(10); // pvt: sonar should be under 40Hz (reduced 100 to 50)
  while (ros::ok()) {

    // Run the read thread sonar
    m750d.m_readData.run();

    // Get bins and beams #.
    nbins = m750d.m_readData.m_osBuffer[0].m_rfm.nRanges;
    nbeams = m750d.m_readData.m_osBuffer[0].m_rfm.nBeams;
    unsigned int id = m750d.m_readData.m_osBuffer[0].m_rfm.pingId;

    // Create pointcloud message from sonar data
    if (nbeams > 0 && nbins > 0 && id > latest_id) {
      latest_id = id;

      // sonar image
      if (m750d.m_readData.m_osBuffer[0].m_rawSize) {
        // if (0){
        sensor_msgs::Image sonar_image;
        sonar_image.header.stamp = ros::Time::now();
        sonar_image.height = nbins;
        sonar_image.width = nbeams;
        sonar_image.encoding = "8UC1";
        // sonar_image.is_bigendian = 0; // default works
        sonar_image.step = nbins;
        sonar_image.data.resize(nbeams * nbins);
        std::copy(m750d.m_readData.m_osBuffer[0].m_pImage,
                  m750d.m_readData.m_osBuffer[0].m_pImage + nbins * nbeams,
                  sonar_image.data.begin());
        // img_pub.publish(sonar_image); // uncomment to bypass the python viewer

        // fire msg
        sonar_oculus::OculusFire fire_msg;
        fire_msg.header.stamp = ros::Time::now();

        fire_msg.mode =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
        fire_msg.gamma =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
        fire_msg.flags = m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
        fire_msg.range = m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
        fire_msg.gain =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
        fire_msg.speed_of_sound =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
        fire_msg.salinity =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;

        // sonar ping
        sonar_oculus::OculusPing ping_msg;
        ping_msg.header.frame_id = frame_str;
        ping_msg.header.stamp = fire_msg.header.stamp;
        ping_msg.ping = sonar_image;
        ping_msg.fire_msg = fire_msg;
        ping_msg.ping_id = id;
        ping_msg.status = m750d.m_readData.m_osBuffer[0].m_rfm.status;
        ping_msg.frequency = m750d.m_readData.m_osBuffer[0].m_rfm.frequency;
        ping_msg.temperature = m750d.m_readData.m_osBuffer[0].m_rfm.temperature;
        ping_msg.pressure = m750d.m_readData.m_osBuffer[0].m_rfm.pressure;
        ping_msg.speed_of_sound =
            m750d.m_readData.m_osBuffer[0].m_rfm.speedOfSoundUsed;

        ping_msg.start_time =
            m750d.m_readData.m_osBuffer[0].m_rfm.pingStartTime;
        ping_msg.bearings.resize(nbeams);
        for (unsigned int i = 0; i < nbeams; ++i)
          ping_msg.bearings[i] = m750d.m_readData.m_osBuffer[0].m_pBrgs[i];
        ping_msg.range_resolution =
            m750d.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
        ping_msg.num_ranges = nbins;
        ping_msg.num_beams = nbeams;

        ping_pub.publish(ping_msg);
      }

    } // if (nbins>0 && nbeams>0 && id>latest_id)

    // Fire sonar (so we sleep while the ping travels)
    m750d.Fire(mode, range, gain, soundspeed, (double)salinity);
    r.sleep();
    // Process ROS events
    ros::spinOnce();
  }

  ROS_INFO("Disconnecting...");
  m750d.Disconnect();

  // Exit
  close(sockUDP);
  close(sockTCP);
  return 0;

} // main
