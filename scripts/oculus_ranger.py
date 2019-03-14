#!/usr/bin/env python
import numpy as np
import cv2
import os.path
from scipy.interpolate import interp1d

# ROS
import rospy
import cv_bridge
from sonar_oculus.msg import OculusPing
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud
from dynamic_reconfigure.server import Server
from sonar_oculus.cfg import OculusParamsConfig

# sonar
from sonar import Sonar

to_rad = lambda bearing: bearing * np.pi / 18000

class Ranger:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()

    # initialize sonar pre-processor
    self.sonar = Sonar()
    cfg_file = 'config/oculus-m1200d.json'
    if os.path.isfile(cfg_file):
      self.sonar.load_config(cfg_file)
    else:
      # no config file found
      rospy.logwarn('No sonar config file found; using defaults')

    # subscribe to topics
    self.ping_sub = rospy.Subscriber('/sonar_oculus_node/ping', OculusPing, self.ping_callback, None, 100)

    # register parameter update callback
    self.cfg_srv = Server(OculusParamsConfig, self.config_callback)

    # advertise topics
    self.cloud_publisher = rospy.Publisher('/sonar_oculus_node/cloud', PointCloud, queue_size=50)
    
  def ping_callback(self,msg):
    """
    Sonar ping callback.
    """
    img = self.bridge.imgmsg_to_cv2(msg.ping, desired_encoding="passthrough")

    # TODO: check if updates to sonar config are needed
    # TODO: save latest config as json
    
    # pre-process ping
    ping = self.sonar.deconvolve(img)
  
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
            cloud_msg.points.append(pt)
                
    self.cloud_publisher.publish(cloud_msg)

  def config_callback(self, config, level):
    """
    Configuration callback.
    """
    self.threshold = config['Threshold']
    return config
 

if __name__ == '__main__':
    rospy.init_node('oculus_ranger')

    sonar_ranger = Ranger()

    rospy.spin()
