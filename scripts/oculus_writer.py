#!/usr/bin/env python
"""
A simple script to dump sonar messages to disk as png images and json files:
 * <unix_epoch_time_us>.json - sonar configuration,
 * cart_<unix_epoch_time_us>.png - image in the Cartesian frame,
 * polar_<unix_epoch_time_us>.png - image in the polar frame.
"""
__author__ = "Pedro Vaz Teixeira"
__email__ = "pvt@mit.edu"
__status__ = "Development"

import json
import numpy as np

import cv2
import rospy
import cv_bridge

from sonar_oculus.msg import OculusPing
from sensor_msgs.msg import Image

from sonar import Sonar

bridge = cv_bridge.CvBridge()

def update_config(msg):
    """Update the sonar object configuration based on the latest message"""
    global oculus
    update = False

    if msg.frequency != oculus.frequency:
        oculus.frequency = msg.frequency
        oculus.fov = msg.bearings[-1] - msg.bearings[0] + 0.0
        oculus.fov *= (np.pi/18000.0)
        update = True

    if msg.fire_msg.range != oculus.max_range:
        oculus.max_range = msg.fire_msg.range
        update = True

    min_range = msg.fire_msg.range - msg.num_ranges*msg.range_resolution

    if min_range != oculus.min_range:
        oculus.min_range = min_range
        update = True

    if msg.num_ranges != oculus.num_bins:
        oculus.num_bins = msg.num_ranges
        update = True

    if update:
        rospy.loginfo('updated sonar config; recomputing lookup table')
        oculus.__computeLookUp__(0.02)
        oculus.printConfig()

def ping_callback(msg):
    """callback function for new Ping messages"""
    rospy.logdebug("received ping")

    cfg = {} # object to store config

    img = bridge.imgmsg_to_cv2(msg.ping, desired_encoding="passthrough")
    timestamp = int(round(msg.header.stamp.to_nsec()*1e-3))
    if timestamp < 1500000000000000: # ~ July 2017
        rospy.logwarn("Got bad timestamp, defaulting to current time!")
        timestamp = int(round(rospy.Time.now().to_nsec()*1e-3))

    filename = str(timestamp)
    cv2.imwrite('polar_'+filename+'.png', img) # will end up inside $HOME/.ros

    # update config
    update_config(msg)
    # rospy.logdebug("width: %s",msg.ping.width)
    # rospy.logdebug("height: %s",msg.ping.height)
    # rospy.logdebug("num_ranges: %s",msg.num_ranges)
    # rospy.logdebug("num_beams: %s",msg.num_beams)

    global oculus
    # convert to cartesian
    img_t = np.copy(img)
    # img_t = np.transpose(img)
    img_xy = oculus.toCart(img_t)
    # write as cartesian
    cv2.imwrite('cart_'+filename+'.png', img_xy) # will end up inside $HOME/.ros

    cfg = {}
    cfg['min_range'] = msg.fire_msg.range - msg.num_ranges*msg.range_resolution
    cfg['max_range'] = msg.fire_msg.range
    cfg['fov'] = msg.bearings[-1] - msg.bearings[0] + 0.0
    cfg['fov'] *= (np.pi/18000.0)

    cfg['num_beams'] = msg.num_beams
    cfg['num_bins'] = msg.num_ranges
    cfg['psf'] = 1   # unknown
    cfg['noise'] = 1 # unknown
    cfg['taper'] = 1 # unknown
    cfg['temperature'] = msg.temperature
    cfg['frequency'] = msg.frequency
    cfg['id'] = msg.ping_id

    cfg['range'] = msg.fire_msg.range
    cfg['gain'] = msg.fire_msg.gain
    cfg['speed_of_sound'] = msg.fire_msg.speed_of_sound
    cfg['salinity'] = msg.fire_msg.salinity
    azi = np.array(msg.bearings).astype(np.float64)
    azi *= (np.pi/18000.0)
    cfg['azimuths'] = azi.tolist()

    with open(filename+'.json', 'w') as fptr:
        json.dump(cfg, fptr, sort_keys=True, indent=2)


if __name__ == '__main__':
    global oculus
    oculus = Sonar()

    # default values based on m750d low-frequency mode
    oculus.min_range = 0.0085
    oculus.max_range = 28.0
    oculus.fov = 2.268928
    oculus.num_beams = 512
    oculus.num_bins = 697
    oculus.noise = 0.02
    oculus.frequency = 750000

    rospy.init_node('oculus_writer')

    ping_sub = rospy.Subscriber('/sonar_oculus_node/ping', OculusPing, ping_callback, None, 10)

    rospy.spin()
