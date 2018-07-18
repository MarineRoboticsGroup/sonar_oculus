#!/usr/bin/env python
import numpy as np
import cv2
from scipy.interpolate import interp1d

import rospy
import cv_bridge
from sonar_oculus.msg import OculusPing
from sensor_msgs.msg import Image, LaserScan
from dynamic_reconfigure.server import Server
from sonar_oculus.cfg import OculusParamsConfig

bridge = cv_bridge.CvBridge()

global threshold,scan_pub

to_rad = lambda bearing: bearing * np.pi / 18000

def ping_callback(msg):
    global threshold,scan_pub

    img = bridge.imgmsg_to_cv2(msg.ping, desired_encoding="passthrough")

    r = np.linspace(0, msg.fire_msg.range,num=msg.num_ranges)
    az = to_rad(np.asarray(msg.bearings, dtype=np.float32))

    # create and publish message
    scan_msg = LaserScan()
    scan_msg.header= msg.header
    scan_msg.angle_min = az[0]
    scan_msg.angle_max = az[-1]
    scan_msg.angle_increment = ( az[-1]-az[0] )/msg.num_beams
    scan_msg.range_min = 0
    scan_msg.range_max = msg.fire_msg.range
    scan_msg.time_increment = 0
    scan_msg.scan_time = 0

    # image is num_ranges x num_beams
    for beam in range(0, len(az)):
        idx = np.argmax(img[:,beam])
        if img[idx,beam] > threshold:
            scan_msg.ranges.append(idx*msg.range_resolution)
            scan_msg.intensities.append(img[idx,beam])
        else:
            scan_msg.ranges.append(-1.0)
            scan_msg.intensities.append(0)

    scan_pub.publish(scan_msg)

def config_callback(config, level):
    global threshold
    threshold = config['Threshold']
    return config

if __name__ == '__main__':
    rospy.init_node('oculus_ranger')
    ping_sub = rospy.Subscriber('/sonar_oculus_node/ping', OculusPing, ping_callback, None, 10)
    server = Server(OculusParamsConfig, config_callback)

    global scan_pub
    scan_pub = rospy.Publisher('/sonar_oculus_node/scan', LaserScan, queue_size=50)

    rospy.spin()
