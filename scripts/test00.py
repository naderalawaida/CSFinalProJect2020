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
from geometry_msgs.msg import Twist



bridge = CvBridge()
frame = None

height= None
width= None

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
move = Twist()


def rotate():
    rate = rospy.Rate(1)
    # What function to call when you ctrl + c    
    rospy.on_shutdown(shutdown)
    move.linear.x = 0
    move.angular.z = 1
    while not rospy.is_shutdown():
        pub.publish(move)
        rate.sleep()

def shutdown():
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        pub.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

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

    redLower = (0, 200, 0)
    redUpper = (255, 255, 255)
    pts = deque(maxlen=64)
    # keep looping
    while not rospy.is_shutdown():
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #print(center)
            #pub.publish(int(M["m01"] / M["m00"]))
            #rospy.loginfo(int(M["m01"] / M["m00"]))
            #rate.sleep()
            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                #cv2.circle(frame, center, 5, (0, 0, 255), -1)
                print('X: ', x)
                if 100 < x < 320:
                    print('Stop Rotation')
        pts.appendleft(center)
        for i in xrange(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
            #cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)


        # show the frame to our screen
        cv2.imshow("Image window", frame)
        cv2.imshow("mask", mask)
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