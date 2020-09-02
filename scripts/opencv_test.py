'''
Required to read in video feed frame by frame from a virtual turtlebot being simulated using gazebo.
Then we detect colors in this image (red, blue, and green) if they are at the center of the image, 
within a rectangle that we have to draw in the center of the frame. 
If one of the colors are detected, the rectangle should change from green to red, 
and if the 's' button is clicked while the rectangle is red (ie it detects a color)
then it should publish to a topic saying that the target was shot
'''


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



hit_pub = rospy.Publisher('Cait/hit', String, queue_size=10)
image_pub = rospy.Publisher("Cait/image_topic_2",Image, queue_size=10)

bridge = CvBridge()
frame = None

height= None
width= None
upper_left=None
lower_right=None

inRange= False
targetHit = False

def ImageCallback(data):
    try:
        global frame
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        #frame = data.data
        global height
        global width

        (h, w) = frame.shape[:2] #w:image-width and h:image-height

        height = h
        width = w

        global centerPoint_x
        global centerPoint_y
        centerPoint_x = w/2
        centerpoint_y= h/2

        global upper_left
        global lower_right
        upper_left = (int(w/4), h)
        lower_right = (int(w*3/4), 0)

    except CvBridgeError as e:
        print(e)

image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,ImageCallback)

def Image_converter():
    # define the lower and upper boundaries of the colors in the HSV color space
    '''
    lower = {'red':(166, 84, 141), 'green':(66, 122, 129), 'yellow':(23, 59, 119)} 
    upper = {'red':(186,255,255), 'green':(86,255,255), 'yellow':(54,255,255)}
    '''
    lower = {'red':(166, 84, 141)}
    upper = {'red':(186,255,255)}



    # grab frames from webcam -> now grab fromm rviz raw_image in subscriber
    #camera = cv2.VideoCapture(0)


    # keep looping
    while True:
     # grab the current frame -> now done by ImageCallback()
        #(grabbed, frame) = camera.read()



        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #for each color in dictionary check object in frame

        #draw rectangle in the center of the frame
        cv2.rectangle(frame, upper_left, lower_right, (0, 255, 0), 2)

        for key, value in upper.items():
            # construct a mask for the color from dictionary`1, then remove any small blobs left in the mask
            kernel = np.ones((9,9),np.uint8)
            mask = cv2.inRange(hsv, lower[key], upper[key])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use it to compute the minimum enclosing circle 
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                #check to see if the point found is at the center of the screen, and if yes change the color of the rectangle
                global inRange

                if (x > int(width/4) & x< int(width*3/4)):
                    if(y > 0 & y < height):
                        cv2.rectangle(frame, upper_left, lower_right, (0, 0, 255), 2)
                        inRange = True
                else:
                    cv2.rectangle(frame, upper_left, lower_right, (0, 255, 0), 2)
                    inRange= False



        # show the frame to our screen
        cv2.imshow("Image window", frame)
        key = cv2.waitKey(1) & 0xFF

        try:
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)

        # if the 's' key is pressed, publish an answer to the topic
        if key == ord("s"):
            if (inRange):
                hit_pub.publish("Target hit!")
            else: 
                hit_pub.publish("Sorry. Nothing was hit.")
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