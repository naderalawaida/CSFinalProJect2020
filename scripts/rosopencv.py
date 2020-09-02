#!/usr/bin/env python
from __future__ import print_function
from collections import deque
import numpy as np
import cv2
import sys
import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()
frame = None

height= None
width= None

def ImageCallback(data):
    try:
        global frame
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        #frame = data.data
        global height
        global width

        (h, w) = frame.shape[:2] #w:image-width and h:image-height

    except CvBridgeError as e:
        print(e)

image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,ImageCallback)

def Image_converter():
    # keep looping
    while True:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
        # show the frame to our screen
        cv2.imshow("Image window", frame)
        key = cv2.waitKey(1) & 0xFF


        if key == ord("q"): #q quits loop
            break   

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  Image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)