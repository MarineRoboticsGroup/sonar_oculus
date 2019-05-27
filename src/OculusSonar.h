/**
 * @file OculusSonar.h
 * @brief Methods to facilitate configuration of Blueprint Subsea Oculus sonar
 *
 * @author Timothy Osedach (tosedach@slb.com)
 */

#ifndef OCULUSSONAR_H
#define OCULUSSONAR_H

#include <arpa/inet.h>
#include <iostream>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <sonar_oculus/OculusParamsConfig.h>
#include <sensor_msgs/Image.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

#include "OculusClient.h"

// Socket Parameters
#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100

// Default Sonar Configuration
#define SONAR_FRAME "sonar"
#define TRIGGER_MODE 0
#define RATE_HZ 7
#define FREQUENCY_MODE 0
#define RANGE 5
#define GAIN 20
#define THRESHOLD 90
#define SOUND_SPEED 1500
#define SALINITY 0

// Struct to contain re-configurable sonar parameters
struct SonarSettings {
    int frequency_mode;  // 0: dev/not used, 1: ~750khz, 2: ~1.2Mhz
    double range;                 // m, limited to 120m in frequency_mode 1 and 40m in frequency_mode 2
    double gain;                  // %
    double threshold;             // intensity threshold
};

// Class to contain sonar configuration parameters
class OculusSonar {
  private:
    OsClientCtrl oculus_control;
    struct in_addr ip_address;
    int sockTCP;
    std::string frame_str;
    int trigger_mode;             // 0: asynchronous, 1: fire on command
    double sound_speed;
    double salinity;             
    double rate_hz;               // Hz
    struct SonarSettings settings;     
    unsigned int nbeams = 0, nbins = 0;
    unsigned int latest_id = 0; // keep track of latest ping to avoid republishing
    dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig> serverParam;
    dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig>::CallbackType f;

  public:
    OculusSonar(void);  // Constructor for SonarConfig class

    // Functions for establishing socket communication with Oculus sonar
    void error(const char *msg);                              // Error handling function
    void get_oculus_ip_address(struct in_addr &ip_address);   // Auto-detects IP address of oculus sonar
    int get_socket(struct in_addr ip_address);                // Returns TCP socket for communicating with sonar
    void fetch_ROS_parameters(ros::NodeHandle nh);            // Method to fetch sonar configuration parameters from the ROS parameter server
    void connect_to_oculus();                                 // Method to establish TCP connection to sonar
    void disconnect_from_oculus();                            // Disconnect from sonar
    // Method to update a subset of parameters of a OculusSonar object
    void update_params(unsigned int frequency_mode, double range, double gain, double threshold); 

    // Dynamic Reconfigure Callback
    void reconfigure_callback(sonar_oculus::OculusParamsConfig &config, uint32_t level);

    void fire_oculus();                      // Method to fire the sonar
    int read_from_oculus();                  // Method to Read from the sonar
    sensor_msgs::Image get_image();          // Method to get a ROS image 
    sonar_oculus::OculusFire get_fire_msg(); // Method to get a ROS fire message
    sonar_oculus::OculusPing get_ping_msg(sensor_msgs::Image sonar_image, sonar_oculus::OculusFire fire_msg); // Method to get a ROS ping message

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
    double get_threshold();
};

#endif