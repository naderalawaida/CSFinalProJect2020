#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import time

# Callback function called whenever
# x-y coordinate received
def drive_callback(data):

    global vel
    ball_x  = data.x
    ball_y  = data.y
    width   = data.z
    
    # Create Twist() instance
    vel = Twist()

    # 
    if ball_x < 0 and ball_y < 0:
        vel.angular.z = 0.5
    else:
        # Determine center-x, normalized deviation from center
        mid_x   = int(width/2)
        delta_x = ball_x - mid_x
        norm_x  = delta_x/width
        mid_y   = int(width/2)
        delta_y = ball_y - mid_y
        norm_y  = delta_y/width

        if norm_x > 0.02:
            #if data.y < 460 or 315 < data.x < 320:
            if norm_y < 0.165:
                print ("delX: {:.3f}. Turn right".format(norm_x))
                vel.angular.z = -0.2
                vel.linear.x=0.1
                print('Y= ', norm_y)
            else:
                print('Stopped')
                vel.angular.z=0
                vel.linear.x=0
                

        elif norm_x < -0.02:
            #if data.y < 460 or 315 < data.x < 320:
            if norm_y < 0.165:
                print ("delX: {:.3f}. Turn left".format(norm_x))
                vel.angular.z = 0.2
                vel.linear.x=0.1
                print('Y= ', norm_y)
            else:
                print('Stopped')
                vel.angular.z=0
                vel.linear.x=0


        if abs(norm_x) < 0.02:
            #if data.y <s 460 or 310 < data.x < 315:
            if norm_y < 0.165:
                print ("delX: {:.3f}. Stay in center".format(norm_x))
                vel.angular.z = 0
                vel.linear.x=0.1
                print('Y= ', norm_y)
            else:
                print('Stopped')
                vel.angular.z=0
                vel.linear.x=0
                rospy.signal_shutdown("Shutdown")

    # publish vel on the publisher
    pub_vel.publish(vel)


if __name__ == '__main__':
    global vel, pub_vel

    # intialize the node
    rospy.init_node('drive_wheel', anonymous=True)

    # subscribe to /ball_location topic to receive coordinates
    img_sub = rospy.Subscriber("/coordinates",Point, drive_callback)
    
    # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    rospy.spin()