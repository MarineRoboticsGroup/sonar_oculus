#!/usr/bin/env python
import numpy as np
import cv2
import os.path
#from scipy.interpolate import interp1d

# ROS
import rospy
import rospkg
import cv_bridge
from sonar_oculus.msg import OculusPing
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud
from dynamic_reconfigure.server import Server
from sonar_oculus.cfg import RangerConfig

# sonar
from sonar import Sonar

to_rad = lambda bearing: bearing * np.pi / 18000

class Ranger:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    # register parameter update callback
    self.cfg_srv = Server(RangerConfig, self.config_callback)

    # import threshold setting from ROS parameter server
    try:
      self.threshold = rospy.get_param(rospy.get_name()+'/threshold')
      rospy.loginfo('Imported ROS parameter \'threshold\': '+str(self.threshold))
    except:
      self.threshold = 90
      rospy.logwarn('Unable to import ROS parameter \'threshold\'.  Using default: '+str(self.threshold))

    # import multibeam sonar model name from ROS parameter server
    try:
      self.config_path = rospy.get_param(rospy.get_name()+'/config_path')
      rospy.loginfo('Imported ROS parameter \'config_path\': '+str(self.config_path))
    except:
      self.config_path = None
      rospy.logwarn('Unable to import ROS parameter \'config_path\'.  Using default: '+str(self.config_path))


    # import multibeam sonar gate limit (high) from ROS parameter server
    try:
      self.gate_high = rospy.get_param(rospy.get_name()+'/gate_high')
      rospy.loginfo('Imported ROS parameter \'gate_high\': '+str(self.gate_high))
    except:
      self.gate_high = 20
      rospy.logwarn('No upper range gate set for oculus sonar. Using default: ', self.gate_high)


    # import multibeam sonar gate limit (low) from ROS parameter server
    try:
      self.gate_low = rospy.get_param(rospy.get_name()+'/gate_low')
      rospy.loginfo('Imported ROS parameter \'gate_low\': '+str(self.gate_low))
    except:
      self.gate_low = 0
      rospy.logwarn('No lower range gate set for oculus sonar. Using default: ', self.gate_low)

    # initialize sonar pre-processor
    self.sonar = Sonar()

    rospack = rospkg.RosPack()
    self.config_path = os.path.join(rospack.get_path('sonar_oculus'), self.config_path)
    
    if os.path.isfile(self.config_path):
      try:
        self.sonar.load_config(self.config_path)
        rospy.loginfo('Successfully loaded sonar config file: '+str(self.config_path))
      except:
        rospy.logerror('Error loading sonar config file: '+str(self.config_path))
    else:
      # no config file found
      rospy.logwarn('No sonar config file found at path, \''+str(self.config_path)+'\'.  Using defaults...')
      cwd = os.getcwd()

    # subscribe to topics
    self.ping_sub = rospy.Subscriber('/sonar/ping', OculusPing, self.ping_callback, None, 100)

    # advertise topics
    self.cloud_publisher = rospy.Publisher('/sonar/cloud', PointCloud, queue_size=50)
    
  def ping_callback(self,msg):
    """
    Sonar ping callback.
    """
    img = self.bridge.imgmsg_to_cv2(msg.ping, desired_encoding="passthrough")

    # TODO: check if updates to sonar config are needed
    # TODO: save latest config as json
    
    # pre-process ping
    #ping = self.sonar.deconvolve(img)
    ping = img

    r = np.linspace(0, msg.fire_msg.range,num=msg.num_ranges)
    az = to_rad(np.asarray(msg.bearings, dtype=np.float32))

    cloud_msg = PointCloud()
    cloud_msg.header = msg.header

    # image is num_ranges x num_beams
    for beam in range(0, len(az)):
        idx = np.argmax(ping[:, beam])
        if ping[idx, beam] > self.threshold:
            # beam range
            br = idx*msg.range_resolution
            # beam azimuth
            ba = -az[beam] # TODO: confirm
            pt = Point()
            pt.x = br*np.cos(ba)
            pt.y = br*np.sin(ba) 
            pt.z = 0.0
            if (br <= self.gate_high) and (br >= self.gate_low):
              cloud_msg.points.append(pt)
                
    self.cloud_publisher.publish(cloud_msg)

  def config_callback(self, config, level):
    """
    Configuration callback.
    """
    self.threshold = config['Threshold']
    self.gate_high = config['Gate_high']
    self.gate_low = config['Gate_low']

    print('Updated Oculus Ranger threshold:', self.threshold)
    print('Updated Oculus Ranger gate_high:', self.gate_high)
    print('Updated Oculus Ranger gate_low:', self.gate_low)
    return config
 

if __name__ == '__main__':
    rospy.init_node('oculus_ranger')

    sonar_ranger = Ranger()

    rospy.spin()
