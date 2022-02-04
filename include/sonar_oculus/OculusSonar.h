/**
 * @file OculusSonar.h
 * @brief Methods to facilitate configuration of Blueprint Subsea Oculus sonar
 *
 * @author Timothy Osedach (tosedach@slb.com)
 * 
 * @note Reference for socket.h:  http://man7.org/linux/man-pages/man2/socket.2.html
 */

#pragma once

// system - C
#include <arpa/inet.h>
#include <iostream>
#include <sys/ioctl.h>
#include <sys/socket.h>

// system - C++
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>

// ros - messages
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

// ros - reconf
#include <sonar_oculus/OculusParamsConfig.h>

// ros - services
#include <sonar_oculus/trigger.h>

#include "OculusClient.h"


// Socket Parameters
#define DATALEN 200000
//#define PORT_UDP 52102
#define PORT_TCP 52100


// Default Sonar Configuration
#define SONAR_FRAME "sonar"
#define TRIGGER_MODE 0
#define RATE_HZ 7
#define FREQUENCY_MODE 0
#define RANGE 5
#define GAIN 20
#define SOUND_SPEED 1500
#define SALINITY 0
#define IP_ADDRESS "192.168.2.4"
//#define POLL_RATE 50

// Struct to contain re-configurable sonar parameters
struct SonarSettings {
    int frequency_mode;  // 0: dev/not used, 1: ~750khz, 2: ~1.2Mhz
    double range;                 // m, limited to 120m in frequency_mode 1 and 40m in frequency_mode 2
    double gain;                  // %
};

// Class to contain sonar configuration parameters
class OculusSonar {
  private:
    OsClientCtrl oculus_control;
    ros::NodeHandle nh;
    ros::Publisher ping_pub;            // publisher for sonar ping message
    ros::ServiceServer trigger_srv;   // service to fire sonar and return sonar ping
    struct in_addr ip_address;

    //struct sockaddr_in serverUDP, clientUDP;
    //int sockUDP;  
    //socklen_t lengthServerUDP, lengthClientUDP;

    struct sockaddr_in serverTCP; 
    int sockTCP = 0;  
    int buf_size = DATALEN;
    int keepalive = 1;
    socklen_t lengthServerTCP; 

    std::string frame_str;
    int trigger_mode;             // 0: asynchronous, 1: fire on command
    double sound_speed;
    double salinity;             
    double rate_hz;               // Hz
    struct SonarSettings settings;     
    unsigned int nbeams = 0, nbins = 0;
    unsigned int latest_id = 0; // keep track of latest ping to avoid republishing 
    dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig> serverParam;
    dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig>::CallbackType f_reconfig;

  public:
    OculusSonar(ros::NodeHandle nh);  // Constructor for SonarConfig class

    // Functions for establishing socket communication with Oculus sonar
    void error(const char *msg);                              // Error handling function
    //void get_oculus_ip_address();   // Auto-detects IP address of oculus sonar
    //int get_socket(struct in_addr ip_address);                // Returns TCP socket for communicating with sonar
    void configure_socket();                      // Configures TCP socket for communicating with sonar
    
    // fetch sonar configuration parameter from the ROS parameter server
    void fetch_ROS_parameters();              
    // establish TCP connection to sonar
    void connect_to_oculus();
    // disconnect from sonar
    void disconnect_from_oculus();
    // update a subset of parameters of a OculusSonar object
    void update_params(unsigned int frequency_mode, double range, double gain); 

    // Dynamic Reconfigure Callback
    void reconfigure_callback(sonar_oculus::OculusParamsConfig &config, uint32_t level);

    // Trigger service Callback
    bool trigger_sonar(sonar_oculus::trigger::Request &req, sonar_oculus::trigger::Response &res);

    void fire_oculus();                      //  fire the sonar
    int read_from_oculus();                  //  Read from the sonar
    sensor_msgs::Image get_image();          //  get a ROS image 
    sonar_oculus::OculusFire get_fire_msg(); //  get a ROS fire message
    sonar_oculus::OculusPing get_ping_msg(sensor_msgs::Image sonar_image, sonar_oculus::OculusFire fire_msg); //  get a ROS ping message
    bool check_for_ping();      //  poll sonar for ping message and publish if one is received
    bool process_ping();
    // Member Get Methods
    struct in_addr get_ip_address();
    unsigned int get_trigger_mode();
    double get_rate_hz();
    double get_sound_speed();
    double get_salinity();
    struct SonarSettings get_settings();
    unsigned int get_frequency_mode();
    double get_range();
    double get_gain();
};
