#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import math

def pid(pt):
    global vel_msg
    global prev_distance_error, prev_angle_error
    global total_distance_error, total_angle_error
    delta = 20
    angle = pt.x
    distance = pt.y
    if angle == 99999:
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        return
    kp_dist, ki_dist, kd_dist = 0.5, 0, 0.25
    kp_angle, ki_angle, kd_angle = 0.003, 0, 0.0001    
    angle_thresh = 160 # we try and keep the angle atmost += 10 pixels from the center
    distance_thresh = 0.7 # We try and ensure that we're almost 1.0 distance from the target

    angle_error = angle_thresh - angle
    print('Angle Error = ')
    print(angle_error)
    distance_error = distance - distance_thresh
    print('Distance Error = ')
    print(distance_error)
    total_distance_error += distance_error
    total_angle_error += angle
        
    v_trans = kp_dist * distance_error + ki_dist * total_distance_error + kd_dist * (distance - prev_distance_error)
    v_angle = kp_angle * angle_error + ki_angle * total_angle_error + kd_angle * (angle - prev_angle_error)

    vel_msg = Twist()
    # Center is (240, 320, 640)
    if abs(distance_error) <= 0.15:
        v_trans = 0
        distance_error = 0
    if abs(angle_error) <= 50:
        v_angle = 0  
        angle_error = 0
    print('Trans. Velocity = ')
    print(v_trans)
    print('Angular Velocity = ')
    print(v_angle)
    vel_msg.linear.x = v_trans
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = v_angle
            
    prev_distance_error, prev_angle_error = distance_error, angle_error
    velocity_publisher.publish(vel_msg)

def Init():
    rospy.init_node('pid', anonymous=True)
    global velocity_publisher
    global prev_distance_error, prev_angle_error
    global total_distance_error, total_angle_error
    prev_distance_error, prev_angle_error, total_distance_error, total_angle_error = 0, 0, 0, 0
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber("/objectLocation", Point, pid, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass
