#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import math
import numpy as np

SAFE_STOP_DISTANCE = 30
LINEAR_VEL = 10

def get_scan(pt):
    x, y, z = pt.x, pt.y, pt.z
    if pt.x == 99999: return 
    # Image Dimensions (240, 320)
    b = x - 160 # Number of pixels from the center of the image to x
    print('Pixels from Center')
    print(b)
    scan = rospy.wait_for_message('scan', LaserScan)
    theta = 31.1 * math.pi / 180 # HFOV for RaspiCam
    phi = (180/np.pi) * math.atan(2 * np.abs(b) * math.tan(theta) / 320)
    # phi += 180
    scan_filter = scan.ranges       
    delta = 5
    if b > 0: phi = 360 - phi
    phi = int(phi)
    for i in range(len(scan_filter)):
        if math.isnan(scan_filter[i]):
            scan_filter[i] = 0 
    filter_slice = np.array(scan_filter[max(0, phi - delta):min(359, phi + delta)])
    print(filter_slice)
    distance = np.mean(filter_slice[filter_slice > 0])
    if math.isnan(distance):
        distance = 0
    # distance = np.median(scan_filter[phi - delta:phi + delta])
    print('Angle', phi)
    print('Distance', distance)
    scan_msg = Point()
    scan_msg.x = pt.x
    scan_msg.y = distance
    object_location.publish(scan_msg)

def Init():
    rospy.init_node('object_position', anonymous=True)
    global object_location
    object_location = rospy.Publisher('objectLocation', Point, queue_size=5)
    rospy.Subscriber("/imageLocation", Point, get_scan, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass
