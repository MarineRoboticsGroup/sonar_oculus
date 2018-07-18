#!/usr/bin/env python
import cv2
import json
import numpy as np

import rospy
import cv_bridge

from sonar_oculus.msg import OculusPing
from sensor_msgs.msg import Image

bridge = cv_bridge.CvBridge()

def ping_callback(msg):
    cfg = {} # object to store config

    img = bridge.imgmsg_to_cv2(msg.ping, desired_encoding="passthrough")
    fn = str(int(round(msg.header.stamp.to_nsec()*1e-3)))
    # rospy.loginfo(fn)
    cv2.imwrite('pol_'+fn+'.png',img) # will end up inside $HOME/.ros

    cfg={}
    cfg['min_range'] = msg.fire_msg.range - msg.num_ranges*msg.range_resolution
    cfg['max_range'] = msg.fire_msg.range
    cfg['fov'] = 2.27 # 130 degrees
    cfg['num_beams'] = msg.num_beams
    cfg['num_bins'] = msg.num_ranges
    cfg['psf'] = 1
    cfg['noise'] = 1
    cfg['temperature'] = msg.temperature
    cfg['frequency'] = msg.frequency
    cfg['id'] = msg.ping_id

    cfg['range'] = msg.fire_msg.range 
    cfg['gain'] = msg.fire_msg.gain 
    cfg['speed_of_sound'] = msg.fire_msg.speed_of_sound 
    cfg['salinity'] = msg.fire_msg.salinity 


    with open(fn+'.json', 'w') as fp:
      json.dump(cfg, fp, sort_keys=True, indent=2)


if __name__ == '__main__':
    rospy.init_node('oculus_writer')

    ping_sub = rospy.Subscriber('/sonar_oculus_node/ping', OculusPing, ping_callback, None, 10)

    rospy.spin()
