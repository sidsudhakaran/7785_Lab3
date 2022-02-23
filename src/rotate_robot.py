#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge() #Bridge converts the image from ros to openCV

def ball_coords(pt):
    global vel_msg
    delta = 20
    x, y, z = pt.x, pt.y, pt.z
    vel_msg = Twist()
    speed = 5 # degrees/sec
    angular_speed = speed * 2 * np.pi / 360
    # Center is (240, 320, 640)
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    if 99999 > pt.x > 240 + delta:
        vel_msg.angular.z = -abs(angular_speed)
    elif pt.x < 240 - delta:
        vel_msg.angular_z = abs(angular_speed)
    velocity_publisher.publish(vel_msg)

def Init():
    rospy.init_node('robot_rotator', anonymous=True)
    global velocity_publisher
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber("/imageLocation", Point, ball_coords, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass


