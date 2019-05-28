#include "OculusSonar.h"

// Implementation for OculusSonar Class

/**
 * @brief Constructor for OculusSonar class
 */
OculusSonar::OculusSonar(void) {

  // Setup dynamic reconfigure server
  this->f = boost::bind(&OculusSonar::reconfigure_callback, this, _1, _2);
  this->serverParam.setCallback(this->f);                     
}

// Functions for establishing socket communication with Oculus sonar

/**
 * @brief Error handling function
 * 
 * @param msg Error message
 */
void OculusSonar::error(const char *msg) {
  perror(msg);
  exit(0);
}

/**
 * @brief Auto-detects IP address of oculus sonar
 * 
 * @param ip_address IP address of target sonar
 */
void OculusSonar::get_oculus_ip_address(struct in_addr &ip_address) {
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
    OculusSonar::error("Error opening UDP listening socket");

  // Bind the UDP socket to address and port, or exit with error
  if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0)
    OculusSonar::error("Error binding UDP listening socket");
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
      
    ROS_INFO(".");
    ros::Duration(1.0).sleep();
  }
}

/**
 * @brief Returns TCP socket for communicating with sonar
 * 
 * @param ip_address IP address of target sonar
 */
int OculusSonar::get_socket(struct in_addr ip_address) { 
  struct sockaddr_in serverTCP; 
  int sockTCP = 0;  
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
    OculusSonar::error("Error opening TCP main socket");

  // Connect to the sonar Server via TCP socket or exit with error
  if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0)
    OculusSonar::error("Error connecting TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0)
    OculusSonar::error("Error increasing RCVBUF for TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0)
    OculusSonar::error("Error keeping alive option set for TCP socket");
  listen(sockTCP, 5);

  return sockTCP;
}

/**
 * @brief Function to fetch sonar configuration parameters from the ROS parameter server
 */
void OculusSonar::fetch_ROS_parameters(ros::NodeHandle nh) {

  //debug
  //std::string ip_address_str = "192.168.2.4";
  //inet_aton(ip_address_str.c_str(), &this->ip_address);
  //this->sound_speed = SOUND_SPEED; 
  //this->salinity = SALINITY;
  //this->frame_str = SONAR_FRAME;
  //this->trigger_mode = TRIGGER_MODE;
  //this->rate_hz = RATE_HZ;
  //this->settings.frequency_mode = FREQUENCY_MODE;
  //this->settings.range = RANGE;
  //this->settings.gain = GAIN;

  std::string ip_address_str;
  if (nh.getParam("ip_address", ip_address_str)) { 
    inet_aton(ip_address_str.c_str(), &this->ip_address);
    ROS_INFO("Imported parameter, \"ip_address\": %s", inet_ntoa(ip_address));
  }
  else { 
    ROS_WARN("Failed to import parameter, \"ip_address\". Will attempt to autodetect sonar...");
    get_oculus_ip_address(this->ip_address);
    ROS_INFO("Oculus sonar detected at %s.", inet_ntoa(this->ip_address));
  }

  if (nh.getParam("/environment/sound_speed", this->sound_speed)) {
    ROS_INFO("Imported parameter, \"/environment/sound_speed\": %f", this->sound_speed);
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"/environment/sound_speed\". Using default (\"%f\").", SOUND_SPEED);
    this->sound_speed = SOUND_SPEED;  // default
  }

  if (nh.getParam("/environment/salinity", this->salinity)) {
    ROS_INFO("Imported parameter, \"/environment/salinity\": %f", this->salinity);
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"/environment/salinity\". Using default (\"%f\").", SALINITY);
    this->salinity = SALINITY;  // default
  }

  if (nh.getParam("frame", this->frame_str)) {
    ROS_INFO("Imported parameter, \"frame\": %s", this->frame_str.c_str());
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"frame\". Using default (\"%s\").", SONAR_FRAME);
    this->frame_str = SONAR_FRAME;  // default
  }

  if (nh.getParam("trigger_mode", this->trigger_mode)) {
    ROS_INFO("Imported parameter, \"trigger_mode\": %i", this->trigger_mode);
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"trigger_mode\". Using default (\"%i\").", TRIGGER_MODE);
    this->trigger_mode = TRIGGER_MODE;  // default
  }

  if (nh.getParam("rate_hz", this->rate_hz)) {
    ROS_INFO("Imported parameter, \"rate_hz\": %f", this->rate_hz);
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"rate_hz\".  Using default (\"%f\").", RATE_HZ);
    this->rate_hz = RATE_HZ;  // default
  }  

  if (nh.getParam("frequency_mode", this->settings.frequency_mode)) {
    ROS_INFO("Imported parameter, \"frequency_mode\": %i", this->settings.frequency_mode);
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"frequency_mode\".  Using default (\"%i\").", FREQUENCY_MODE);
    this->settings.frequency_mode = FREQUENCY_MODE;  // default
  }  

  if (nh.getParam("range", this->settings.range)) {
    ROS_INFO("Imported parameter, \"range\": %f", this->settings.range);
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"range\".  Using default (\"%f\").", RANGE);
    this->settings.range = RANGE;  // default
  }  

  if (nh.getParam("gain", this->settings.gain)) {
    ROS_INFO("Imported parameter, \"gain\": %f", this->settings.gain);
  } 
  else {
    ROS_ERROR("Failed to import parameter, \"gain\".  Using default (\"%f\").", GAIN);
    this->settings.gain = GAIN;  // default
  }  
}

/**
 * @brief Function to establish TCP connection with the sonar
 */
void OculusSonar::connect_to_oculus() {
  // Setup TCP connection with sonar
  std::cout << "Getting socket!\n";   // magically needed
  this->sockTCP = OculusSonar::get_socket(this->ip_address);
  std::cout << "Using socket: " << this->sockTCP << " for sonar at " << inet_ntoa(this->ip_address) << "\n";
  this->oculus_control.m_readData.m_pSocket = &this->sockTCP;   // Pass the socket to the control

  this->oculus_control.Connect();
  ROS_INFO("Connected to sonar!");
}

/**
 * @brief Function to disconnect from the sonar
 */
void OculusSonar::disconnect_from_oculus() {
  this->oculus_control.Disconnect();
  close(this->sockTCP);  // close sockets
} 

/**
 * @brief Function to update sonar configuration parameters
 * 
 * @param frequency_mode   0: dev/not used, 1: ~750khz, 2: ~1.2Mhz
 * @param range            Range in m, limited to 120m in frequency_mode 1 and 40m in frequency_mode 2
 * @param gain             %
 */
void OculusSonar::update_params(unsigned int frequency_mode, double range, double gain) {
  this->settings.frequency_mode = frequency_mode;
  this->settings.range = range;
  this->settings.gain = gain;
}

/**
 * @brief Dynamic reconfigure server callback
 * 
 * @param config 
 * @param level 
 */
void OculusSonar::reconfigure_callback(sonar_oculus::OculusParamsConfig &config, uint32_t level) {
  this->settings.frequency_mode = config.Mode;
  this->settings.gain = config.Gain;
  this->settings.range = config.Range;

  ROS_INFO("Reconfigure Request: mode: %i, gain: %i, range: %i", 
           config.Mode, config.Gain, config.Range);
}

/**
 * @brief Function to fire the sonar
 */
void OculusSonar::fire_oculus() {
  this->oculus_control.Fire(this->settings.frequency_mode, 
                            this->settings.range, 
                            this->settings.gain, 
                            this->sound_speed, 
                            this->salinity);
}

/**
 * @brief Function to read the most recent data from the sonar
 */
int OculusSonar::read_from_oculus() {
  // run the read thread sonar
  this->oculus_control.m_readData.run();

  // get bins and beams #.
  unsigned int nbins = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.nRanges;
  unsigned int nbeams = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.nBeams;
  unsigned int id = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.pingId;

  // create pointcloud message from sonar data
  if (nbeams > 0 && nbins > 0 && id > this->latest_id) {
    this->latest_id = id;
    this->nbins = nbins;
    this->nbeams = nbeams;
    //ROS_INFO("Latest ID: %i", latest_id);
    
    if (this->oculus_control.m_readData.m_osBuffer[0].m_rawSize) {
      return 1;
    }
    else {
      return 0;
    }
  }
}

/**
 * @brief Method to get a ROS image 
 */
sensor_msgs::Image OculusSonar::get_image() {
  sensor_msgs::Image sonar_image;
  sonar_image.header.stamp = ros::Time::now();
  sonar_image.height = this->nbins;
  sonar_image.width = this->nbeams;
  sonar_image.encoding = "8UC1";
  // sonar_image.is_bigendian = 0; // default works
  sonar_image.step = this->nbins;
  sonar_image.data.resize(this->nbeams * this->nbins);
  std::copy(this->oculus_control.m_readData.m_osBuffer[0].m_pImage,
            this->oculus_control.m_readData.m_osBuffer[0].m_pImage + this->nbins * this->nbeams,
            sonar_image.data.begin());
  return sonar_image;
}

/**
 * @brief Method to get a ROS fire message
 */
sonar_oculus::OculusFire OculusSonar::get_fire_msg() {
  sonar_oculus::OculusFire fire_msg;
  fire_msg.header.stamp = ros::Time::now();

  fire_msg.mode = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
  fire_msg.gamma = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
  fire_msg.flags = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
  fire_msg.range = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
  fire_msg.gain = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
  fire_msg.speed_of_sound = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
  fire_msg.salinity = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;
  return fire_msg;
}

/**
 * @brief Method to get a ROS ping message
 */
sonar_oculus::OculusPing OculusSonar::get_ping_msg(sensor_msgs::Image sonar_image, sonar_oculus::OculusFire fire_msg) {
  sonar_oculus::OculusPing ping_msg;
  ping_msg.header.frame_id = this->frame_str;
  ping_msg.header.stamp = fire_msg.header.stamp;
  ping_msg.ping = sonar_image;
  ping_msg.fire_msg = fire_msg;
  ping_msg.ping_id = this->latest_id;
  ping_msg.status = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.status;
  ping_msg.frequency = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.frequency;
  ping_msg.temperature = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.temperature;
  ping_msg.pressure = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.pressure;
  ping_msg.speed_of_sound = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.speedOfSoundUsed;

  ping_msg.start_time = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.pingStartTime;
  ping_msg.bearings.resize(this->nbeams);
  for (unsigned int i = 0; i < this->nbeams; ++i)
    ping_msg.bearings[i] = this->oculus_control.m_readData.m_osBuffer[0].m_pBrgs[i];
  ping_msg.range_resolution = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
  ping_msg.num_ranges = this->nbins;
  ping_msg.num_beams = this->nbeams;
  return ping_msg;
}


// Member Get Methods

/**
 * @brief Function to get sonar IP address
 */
struct in_addr OculusSonar::get_ip_address() {
  return this->ip_address;
}

/**
 * @brief Function to get sonar trigger_mode configuration (0: asynchronous, 1: fire on command)
 */
unsigned int OculusSonar::get_trigger_mode() {
  return this->trigger_mode;
}

/**
 * @brief Function to get sonar ping rate configuration (Hz)
 */
double OculusSonar::get_rate_hz() {
  return this->rate_hz;
}

/**
 * @brief Function to return configured sound speed
 */
double OculusSonar::get_sound_speed() {
  return this->sound_speed;
}

/**
 * @brief Function to return configured salinity
 */
double OculusSonar::get_salinity() {
  return this->salinity;
}

/**
 * @brief Function to return SonarSettings struct, containing all reconfigurable parameters
 */
struct SonarSettings OculusSonar::get_settings() {
  return this->settings;
}

/**
 * @brief Function to return frequency mode settings (0: dev/not used, 1: ~750khz, 2: ~1.2Mhz)
 */
unsigned int OculusSonar::get_frequency_mode() {
  return this->settings.frequency_mode;
}

/**
 * @brief Function to return range setting (in meters)
 */
double OculusSonar::get_range() {
  return this->settings.range;
}

/**
 * @brief Function to return gain setting (%)
 */
double OculusSonar::get_gain() {
  return this->settings.gain;
}