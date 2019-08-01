#include <sonar_oculus/OculusSonar.h>

// Implementation for OculusSonar Class

/**
 * @brief Constructor for OculusSonar class
 * 
 *  * @param msg nh ROS node handle
 */
OculusSonar::OculusSonar(ros::NodeHandle nh) {
  // Initialize ROS node handle and parameters
  this->nh = nh;
  this->fetch_ROS_parameters();     // Get parameters from ROS parameters server (or default)
  this->ping_pub = this->nh.advertise<sonar_oculus::OculusPing>("ping", 1);

  // Setup trigger service
  this->trigger_srv = this->nh.advertiseService("trigger", &OculusSonar::trigger_sonar, this);

  // Setup dynamic reconfigure server
  this->f_reconfig = boost::bind(&OculusSonar::reconfigure_callback, this, _1, _2);
  this->serverParam.setCallback(this->f_reconfig);   
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
 * @brief Function to fetch sonar configuration parameters from the ROS parameter server
 */
void OculusSonar::fetch_ROS_parameters() {

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
  if (this->nh.getParam("ip_address", ip_address_str)) { 
    inet_aton(ip_address_str.c_str(), &this->ip_address);
    ROS_INFO("Imported parameter, \"ip_address\": %s", inet_ntoa(ip_address));
  }
  else { 
    ROS_WARN("Failed to import parameter, \"ip_address\". Using default (\"%s\").", IP_ADDRESS);
    ip_address_str = IP_ADDRESS;
    inet_aton(ip_address_str.c_str(), &this->ip_address);
  }

  if (this->nh.getParam("/environment/sound_speed", this->sound_speed)) {
    ROS_INFO("Imported parameter, \"/environment/sound_speed\": %f", this->sound_speed);
  } 
  else {
    ROS_WARN("Failed to import parameter, \"/environment/sound_speed\". Using default (\"%f\").", SOUND_SPEED);
    this->sound_speed = SOUND_SPEED;  // default
  }

  if (this->nh.getParam("/environment/salinity", this->salinity)) {
    ROS_INFO("Imported parameter, \"/environment/salinity\": %f", this->salinity);
  } 
  else {
    ROS_WARN("Failed to import parameter, \"/environment/salinity\". Using default (\"%f\").", SALINITY);
    this->salinity = SALINITY;  // default
  }

  if (this->nh.getParam("frame", this->frame_str)) {
    ROS_INFO("Imported parameter, \"frame\": %s", this->frame_str.c_str());
  } 
  else {
    ROS_WARN("Failed to import parameter, \"frame\". Using default (\"%s\").", SONAR_FRAME);
    this->frame_str = SONAR_FRAME;  // default
  }

  if (this->nh.getParam("trigger_mode", this->trigger_mode)) {
    ROS_INFO("Imported parameter, \"trigger_mode\": %i", this->trigger_mode);
  } 
  else {
    ROS_WARN("Failed to import parameter, \"trigger_mode\". Using default (\"%i\").", TRIGGER_MODE);
    this->trigger_mode = TRIGGER_MODE;  // default
  }

  if (this->nh.getParam("rate_hz", this->rate_hz)) {
    ROS_INFO("Imported parameter, \"rate_hz\": %f", this->rate_hz);
  } 
  else {
    ROS_WARN("Failed to import parameter, \"rate_hz\".  Using default (\"%f\").", RATE_HZ);
    this->rate_hz = RATE_HZ;  // default
  }  

  if (this->nh.getParam("frequency_mode", this->settings.frequency_mode)) {
    ROS_INFO("Imported parameter, \"frequency_mode\": %i", this->settings.frequency_mode);
  } 
  else {
    ROS_WARN("Failed to import parameter, \"frequency_mode\".  Using default (\"%i\").", FREQUENCY_MODE);
    this->settings.frequency_mode = FREQUENCY_MODE;  // default
  }  

  if (this->nh.getParam("range", this->settings.range)) {
    ROS_INFO("Imported parameter, \"range\": %f", this->settings.range);
  } 
  else {
    ROS_WARN("Failed to import parameter, \"range\".  Using default (\"%f\").", RANGE);
    this->settings.range = RANGE;  // default
  }  

  if (this->nh.getParam("gain", this->settings.gain)) {
    ROS_INFO("Imported parameter, \"gain\": %f", this->settings.gain);
  } 
  else {
    ROS_WARN("Failed to import parameter, \"gain\".  Using default (\"%f\").", GAIN);
    this->settings.gain = GAIN;  // default
  }  
}


/**
 * @brief Function to establish TCP connection with the sonar
 */
void OculusSonar::connect_to_oculus() {
  // Setup TCP connection with sonar
  std::cout << "Getting socket!\n";   // magically needed
  this->configure_socket();
  std::cout << "Using socket: " << this->sockTCP << " for sonar at " << inet_ntoa(this->ip_address) << "\n";
  this->oculus_control.m_readData.m_pSocket = &this->sockTCP;   // Pass the socket to the control

  this->oculus_control.Connect();
  ROS_INFO("Connected to sonar!");
}


/**
 * @brief Returns TCP socket for communicating with sonar
 * 
 * @param ip_address IP address of target sonar
 */
void OculusSonar::configure_socket() { 
  
  this->lengthServerTCP = sizeof(this->serverTCP);
  bzero((char *)&this->serverTCP, this->lengthServerTCP);
  this->serverTCP.sin_family = AF_INET;
  this->serverTCP.sin_addr.s_addr = this->ip_address.s_addr;
  this->serverTCP.sin_port = htons(PORT_TCP);

  // Create the TCP socket for main communication or exit
  this->sockTCP = socket(AF_INET, SOCK_STREAM, 0);
  std::cout << "socket:" << this->sockTCP << std::endl;
  if (this->sockTCP < 0)
    OculusSonar::error("Error opening TCP main socket");

  // Connect to the sonar Server via TCP socket or exit with error
  if (connect(this->sockTCP, (struct sockaddr *)&this->serverTCP, this->lengthServerTCP) < 0)
    OculusSonar::error("Error connecting TCP socket");
  if (setsockopt(this->sockTCP, SOL_SOCKET, SO_RCVBUF, &this->buf_size, sizeof(this->buf_size)) < 0)
    OculusSonar::error("Error increasing RCVBUF for TCP socket");
  if (setsockopt(this->sockTCP, SOL_SOCKET, SO_KEEPALIVE, &this->keepalive, sizeof(this->keepalive)) < 0)
    OculusSonar::error("Error keeping alive option set for TCP socket");
  listen(this->sockTCP, 5);
}


/**
 * @brief Function to disconnect from the sonar
 */
void OculusSonar::disconnect_from_oculus() {
  this->oculus_control.Disconnect();
  // close sockets
  close(this->sockTCP);  
  //close(this->sockUDP);
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
 * @brief Method to poll sonar for ping message and publish if one is received
 */
bool OculusSonar::process_ping() {
  //std::cout << "reading from sonar..." << std::endl;
  this->oculus_control.m_readData.run();
  // get bins and beams #.
  this->nbins = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.nRanges;
  this->nbeams = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.nBeams;
  unsigned int id = this->oculus_control.m_readData.m_osBuffer[0].m_rfm.pingId;
  //std::cout << "latest_id:" << this->latest_id << " id: " << id << " nbins:" << this->nbins << " nbeams:" << this->nbeams << std::endl;  // debug
  // create pointcloud message from sonar data
  if (this->nbeams > 0 && this->nbins > 0 && id > this->latest_id) {
    //std::cout << "id " << id << " is greater than this->latest_id " << this->latest_id << std::endl; 
    this->latest_id = id;

    //ROS_INFO("Latest ID: %i", this->latest_id); // debug
    if (this->oculus_control.m_readData.m_osBuffer[0].m_rawSize) {

      //std::cout << "successfully read valid data from sonar." << std::endl;
      sensor_msgs::Image sonar_image = this->get_image();        // image msg
      //std::cout << "generated new sonar image..." << std::endl;
      sonar_oculus::OculusFire fire_msg = this->get_fire_msg();  // fire msg
      //std::cout << "generated new fire message..." << std::endl;
      sonar_oculus::OculusPing ping_msg = this->get_ping_msg(sonar_image, fire_msg);  // sonar ping
      //std::cout << "generated new ping message , publishing..." << std::endl;
      this->ping_pub.publish(ping_msg);
      return true;
      }
  } 
  return false;
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


/**
 * @brief ROS service to trigger sonar ping acquisition
 */ 
bool OculusSonar::trigger_sonar(sonar_oculus::trigger::Request &req, sonar_oculus::trigger::Response &res) {

  if (this->process_ping() == true) {
    //std::cout << "ping detected..." << std::endl;
    this->fire_oculus();  
  }

  return true;
}


  // TO DO:  IMPLEMENTATION OF THIS SERVICE
  /* this->fire_oculus();      // fire sonar
  ros::Rate poll_rate(POLL_RATE);
  while (ros::ok()) {
    if (this->check_for_ping() == true)
      break;
    poll_rate.sleep();
    ros::spinOnce();  // tpo: is this needed?
  } */
  //std::cout << "TRIGGER CALLBACK" << std::endl;



/**
 * @brief Auto-detects IP address of oculus sonar
 * 
 * @param ip_address IP address of target sonar
 */
/* void OculusSonar::get_oculus_ip_address() {
  std::cout << "BEGINNING CONNECTION" << std::endl;
  // Clear and initialize values of server and client network info
  this->lengthServerUDP = sizeof(this->serverUDP);
  bzero((char *)&this->serverUDP, this->lengthServerUDP);
  this->serverUDP.sin_family = AF_INET;
  this->serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
  this->serverUDP.sin_port = htons(PORT_UDP);
  this->lengthClientUDP = sizeof(this->clientUDP);

  // Create the UDP listening socket or exit
  this->sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->sockUDP < 0)
    OculusSonar::error("Error opening UDP listening socket");

  // Bind the UDP socket to address and port, or exit with error
  if (bind(this->sockUDP, (struct sockaddr *)&this->serverUDP, this->lengthServerUDP) < 0)
    OculusSonar::error("Error binding UDP listening socket");
  listen(this->sockUDP, 5);

  int64_t bytesAvailable = 0;
  OculusStatusMsg osm;

  while (true) {
    ioctl(sockUDP, FIONREAD, &bytesAvailable);
    if (bytesAvailable > 0) {
      unsigned bytesRead = read(this->sockUDP, (char *)&osm, bytesAvailable);

      ip_address.s_addr = osm.ipAddr;
      printf("The IP address is %s\n", inet_ntoa(ip_address));
      this->ip_address = ip_address;
      this->get_socket();
      break;
    }
      
    ROS_INFO(".");
    ros::Duration(1.0).sleep();
  }
} */

/**
 * @brief Method to poll sonar for ping message and publish if one is received
 */
/* bool OculusSonar::check_for_ping() {
  if (this->read_from_oculus() > 0) {
      sensor_msgs::Image sonar_image = this->get_image();        // image msg
      sonar_oculus::OculusFire fire_msg = this->get_fire_msg();  // fire msg
      sonar_oculus::OculusPing ping_msg = this->get_ping_msg(sonar_image, fire_msg);  // sonar ping

      this->ping_pub.publish(ping_msg);
      return true;
  } 
  else {
    return false;
  }

}
*/


/**
 * @brief Function to read the most recent data from the sonar
 */
/* int OculusSonar::read_from_oculus() {
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
} */
