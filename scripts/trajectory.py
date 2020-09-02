#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('vel_publisher')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
move = Twist()


def rotate():
    rate = rospy.Rate(1)
    # What function to call when you ctrl + c    
    rospy.on_shutdown(shutdown)
    move.linear.x = 1
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

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass