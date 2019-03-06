#!/usr/bin/env python
import numpy as np
import cv2
from scipy.interpolate import interp1d

import rospy
import cv_bridge
from sonar_oculus.msg import OculusPing
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud
from dynamic_reconfigure.server import Server
from sonar_oculus.cfg import OculusParamsConfig

bridge = cv_bridge.CvBridge()

global threshold
global cloud_pub

# 
to_rad = lambda bearing: bearing * np.pi / 18000

def ping_callback(msg):
    global threshold
    global cloud_pub

    img = bridge.imgmsg_to_cv2(msg.ping, desired_encoding="passthrough")

    # TODO: use multibeam here!

    r = np.linspace(0, msg.fire_msg.range,num=msg.num_ranges)
    az = to_rad(np.asarray(msg.bearings, dtype=np.float32))

    cloud_msg = PointCloud()
    cloud_msg.header = msg.header

    # image is num_ranges x num_beams
    for beam in range(0, len(az)):
        idx = np.argmax(img[:,beam])
        if img[idx,beam] > threshold:
            br = idx*msg.range_resolution
            ba = -az[beam] # TODO: confirm
            pt = Point()
            pt.x = br*np.cos(ba)
            pt.y = br*np.sin(ba) 
            pt.z = 0.0
            cloud_msg.points.append(pt)
                
    cloud_pub.publish(cloud_msg)

    
def config_callback(config, level):
    global threshold
    threshold = config['Threshold']
    return config

if __name__ == '__main__':
    rospy.init_node('oculus_ranger')
    ping_sub = rospy.Subscriber('/sonar_oculus_node/ping', OculusPing, ping_callback, None, 10)
    server = Server(OculusParamsConfig, config_callback)

    global cloud_pub
    cloud_pub = rospy.Publisher('/sonar_oculus_node/cloud',PointCloud, queue_size=50)

    rospy.spin()
