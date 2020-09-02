#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
#from apriltag_ros.msg import AprilTagDetectionArray
from moveit_commander.conversions import pose_to_list
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
from nav_msgs.msg import Odometry
from math import radians
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



PI = 3.1415926535897

rospy.init_node('tb4_node', anonymous=True) 
#rospy.init_node('moveit_routine_fix', anonymous=True)    

#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
r = rospy.Rate(50)

    # MOVEIT INIT
    
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander("/tb3_4/robot_description","tb3_4")
#scene = moveit_commander.PlanningSceneInterface("/tb3_4/robot_description","tb3_4")
group_arm = "arm"
group_name = "gripper"
group = moveit_commander.MoveGroupCommander(group_name,"/tb3_4/robot_description","tb3_4")
grouparm = moveit_commander.MoveGroupCommander(group_arm,"/tb3_4/robot_description","tb3_4")




def Dorotation():
    #if you set a goal while it's docked it tends to run into the docking station while turning.  Tell it to back up a little before initiliazing goals.
        proactive_charging_at_dock_station = False
        cmd_vel = rospy.Publisher('tb3_4/cmd_vel', Twist, queue_size=10)
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.1 m/s
        move_cmd.linear.x = 0
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0.1

        r = rospy.Rate(10);
        # as long as you haven't ctrl + c keeping doing...
        temp_count = 0
        #go back at 0.1 m/s for 2 seconds
        while (not rospy.is_shutdown() and temp_count < 20):
            # publish the velocity
            cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            temp_count = temp_count + 1
            r.sleep()
        #make sure TurtleBot stops by sending a default Twist()
        cmd_vel.publish(Twist())
        return True



def goto(pos, quat):

            # Send a goal
            # DEFINE PARAMETERS
    group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
    group.set_planning_time(5)                      # TIME TO PLANNING
    grouparm.set_goal_position_tolerance(0.1)       # GOAL TOLERANCE
    grouparm.set_planning_time(5)


    goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
    #rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
    move_base = actionlib.SimpleActionClient("/tb3_4/move_base", MoveBaseAction)
    rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
    move_base.wait_for_server(rospy.Duration(5))


    #function starts here
    goal_sent = True
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

    # Start moving
    move_base.send_goal(goal)

    # Allow TurtleBot up to 60 seconds to complete task
    success = move_base.wait_for_result(rospy.Duration(60)) 

    state = move_base.get_state()
    result = False

    if success and state == GoalStatus.SUCCEEDED:
            # We made it!
        result = True
    else:
        move_base.cancel_goal()
        print('Goal is cancelled')

    goal_sent = False
    return result


def rotate():
  #Starts a new node
  #rospy.init_node('robot_cleaner', anonymous=True)
  velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  vel_msg = Twist()

  # Receiveing the user's input
  #print("Let's rotate your robot")
  #speed = input("Input your speed (degrees/sec):")
  #angle = input("Type your distance (degrees):")
  #clockwise = input("Clockwise?: ") #True or false


  angle = 90
  clockwise = 1

  #Converting from angles to radians
  #angular_speed = speed*2*PI/360
  angular_speed = 30*2*PI/360
  relative_angle = angle*2*PI/360


  #We wont use linear components
  vel_msg.linear.x=0
  vel_msg.linear.y=0
  vel_msg.linear.z=0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0


# Checking if our movement is CW or CCW
  if clockwise:
    #vel_msg.angular.z = -abs(angular_speed)
    vel_msg.angular.z = 0.5
  else:
    vel_msg.angular.z = 0.5
    #vel_msg.angular.z = abs(angular_speed)

 # Setting the current time for distance calculus
  t0 = rospy.Time.now().to_sec()
  current_angle = 0

  temp_count = 0

  while current_angle < relative_angle:
      velocity_publisher.publish(vel_msg)
      t1 = rospy.Time.now().to_sec()
      current_angle = angular_speed*(t1-t0)
      print('current_angle', current_angle)
      print('relative_angle ', relative_angle)
      r.sleep()
      if current_angle > 2.0:
        print('Break')
        break
  
  #Forcing our robot to stop
  vel_msg.angular.z = 0
  velocity_publisher.publish(vel_msg)
  rospy.spin()

  
  #make sure TurtleBot stops by sending a default Twist()
  velocity_publisher.publish(Twist())
  rospy.sleep(1)

  return True  

# Callback function called whenever
# x-y coordinate received
def func(_static={'counter': 0},add=1):
        _static['counter'] += add
            
        return _static['counter']


def myfunc():
    myfunc.counter += 1
    return myfunc.counter


myfunc.counter = 0


def drive_callback(data, args):
    #status = True


    x = args[0]
    y = args[1]


    global vel, pub_vel
    ball_x  = data.x
    ball_y  = data.y
    width   = data.z
              
             # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('tb3_4/cmd_vel', Twist, queue_size=5)
              # Create Twist() instance
    vel = Twist()
                # Determine center-x, normalized deviation from center
    mid_x   = int(width/2)
    delta_x = ball_x - mid_x
    norm_x  = delta_x/width
    mid_y   = int(width/2)
    delta_y = ball_y - mid_y
    norm_y  = delta_y/width

    rospy.on_shutdown(shutdown)
    angle = 90
    clockwise = 1

              #Converting from angles to radians
              #angular_speed = speed*2*PI/360
    angular_speed = 30*2*PI/360
    relative_angle = angle*2*PI/360


              #We wont use linear components
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0
    vel.angular.x = 0
    vel.angular.y = 0


            # Checking if our movement is CW or CCW
            
    vel.angular.z = 0.2
                    

    r = rospy.Rate(30);

    iterate = -100

    

    r1 = rospy.Rate(80);

    temp_count = 0
    final_temp = 0

    print('ball_x first value: ', ball_x)
    
    if iterate < 0:

        print('ballx initial value ', ball_x)
        while (not rospy.is_shutdown() and final_temp < 1):
            #print('Final Search')
            #print('ball_x first value: ', ball_x)
            
                    # as long as you haven't ctrl + c keeping doing...
            
                
                    #go back at 0.1 m/s for 2 seconds
            if ball_x < 0 and ball_y < 0:
                #print('Searching...')
                while (not rospy.is_shutdown() and temp_count < 1):
                         # publish the velocity
                    pub_vel.publish(vel)
                                # wait for 0.1 seconds (10 HZ) and publish again
                    temp_count = temp_count + 1
                    
                    r.sleep()
                            
                        #make sure TurtleBot stops by sending a default Twist()
                pub_vel.publish(Twist())

                        #print('Ball_x', ball_x)
                        
            else:
                a = func(add=-1)
                print('a :', a)
                #print('Box Found!')
                if norm_x > 0.02:
                    print ("delX: {:.3f}. Turn right".format(norm_x))
                    vel.angular.z = -0.2
                    vel.linear.x=0.1
                    print('Y= ', norm_y)  

                elif norm_x < -0.02:
                    print ("delX: {:.3f}. Turn left".format(norm_x))
                    vel.angular.z = 0.2
                    vel.linear.x=0.1
                    print('Y= ', norm_y)
                            
                else:
                    if norm_y < 0.105:
                        print ("delX: {:.3f}. Stay in center".format(norm_x))
                        vel.angular.z = 0
                        vel.linear.x=0.1
                        print('Y= ', norm_y)
                    else:
                        a = func(add=0)
                        print('Stopped')
                        vel.angular.z=0
                        vel.linear.x=0
                        pub_vel.publish(Twist())
                        open_grip()
                        pick_up()
                        grab_up()
                        close_grip() 
                        pick()
                        go_to_station()
                        open_grip()
                        go_to_point(x,y)
                        print('End main loop')
                        
                        
                #rospy.signal_shutdown("Shutdown")        
                #continue        
                       
                # publish vel on the publisher
            print('End of last main loop')
            pub_vel.publish(vel)
            final_temp = final_temp + 1
            a = func(add = 1)
            #a = funccounter()
            print('a ', a)
            r1.sleep()

            if a > 1200:
                print('Search Mission Aborted')
                pub_vel.publish(Twist())
                rospy.spin()
                #rospy.signal_shutdown("Shutdown")
                                

        #make sure TurtleBot stops by sending a default Twist()
        pub_vel.publish(Twist())
        print('Final Ball_x', ball_x)




    '''    
            #go back at 0.1 m/s for 2 seconds
    if ball_x < 0 and ball_y < 0:
        print('Searching...')
        while (not rospy.is_shutdown() and temp_count < 1):
                # publish the velocity
            pub_vel.publish(vel)
                # wait for 0.1 seconds (10 HZ) and publish again
            temp_count = temp_count + 1
            a = func()
            print(a)
            r.sleep()
            if a > 100:
                    #print('Break')
                #break
                #status = False
                print('Search Mission Aborted')
                pub_vel.publish(Twist())
                rospy.signal_shutdown("Shutdown")
                
            
            #make sure TurtleBot stops by sending a default Twist()
        pub_vel.publish(Twist())

            #print('Ball_x', ball_x)
        
    else:
        print('Box Found!')
        if norm_x > 0.02:
            print ("delX: {:.3f}. Turn right".format(norm_x))
            vel.angular.z = -0.2
            vel.linear.x=0.1
            print('Y= ', norm_y)  

        elif norm_x < -0.02:
            print ("delX: {:.3f}. Turn left".format(norm_x))
            vel.angular.z = 0.2
            vel.linear.x=0.1
            print('Y= ', norm_y)
            
        else:
            if norm_y < 0.103:
                print ("delX: {:.3f}. Stay in center".format(norm_x))
                vel.angular.z = 0
                vel.linear.x=0.1
                print('Y= ', norm_y)
            else:
                print('Stopped')
                vel.angular.z=0
                vel.linear.x=0
                pub_vel.publish(Twist())
                open_grip()
                pick_up()
                grab_up()
                close_grip() 
                pick()
                go_to_station()
                open_grip()
                got_to_point(x,y)
                rospy.signal_shutdown("Shutdown")
                
               
        # publish vel on the publisher
    pub_vel.publish(vel)
    '''    


def shutdown():
    rospy.loginfo("Stop")


def find(x,y):
    # intialize the node
    #rospy.init_node('drive_wheel', anonymous=True)
    #subscribe to /ball_location topic to receive coordinates
    #rospy.init_node('moveit_routine_fix', anonymous=True)
    img_sub = rospy.Subscriber("/coordinates04",Point, drive_callback, (x, y))
    rospy.spin()


def pick(j = pi/2):
    joint_goal = grouparm.get_current_joint_values()
    joint_goal[2] = -j/2
    joint_goal[1] = 0/2
    joint_goal[3] = 0/2
    grouparm.go(joints=joint_goal, wait=True)
    grouparm.stop()
    grouparm.clear_pose_targets()    


def pick_up(j = pi/2):
    joint_goal = grouparm.get_current_joint_values()

    joint_goal[1] = -j/2 - 0.30
    joint_goal[2] = j/2 + 0.30
    joint_goal[3] = j/2 - 0.30
    grouparm.go(joints=joint_goal, wait=True)
    grouparm.stop()
    grouparm.clear_pose_targets()

def grab_up(j = pi/2):
    joint_goal = grouparm.get_current_joint_values()
        
    joint_goal[1] = j/2 + 0.60
    joint_goal[2] = -j/2
    joint_goal[3] = -j/2 + 0.80
    grouparm.go(joints=joint_goal, wait=True)
    grouparm.stop()
    grouparm.clear_pose_targets()
    print "Grab"   


def open_grip():
    print "Open Grip"
    jv = group.get_current_joint_values()  
    jv = [0.019,0.019]
    print(jv)
    group.go(joints=jv, wait=True)
    group.stop()


def close_grip():
    print "Close Grip"
    jv = group.get_current_joint_values() 
    jv = [-0.010,-0.010]
    print(jv)
    group.go(joints=jv, wait=True)
    group.stop()

def go_to_station():
    position = {'x': 3.3, 'y' : 2.5, 'z' : 0.}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    goto(position, quaternion)

def go_to_point(x,y):
    position = {'x': x, 'y' : y}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    goto(position, quaternion)

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo('x: {}, y:{}'.format(x,y))

def main():
    rospy.Subscriber('/odom', Odometry, callback, )
    rospy.spin()



if __name__ == '__main__':

    #Dorotation()
    #main()
    #try:
    #rotate()
    #got_to_point(4.5,0.38)
    find(0.0,0.0)
    #close_grip()
    #go_to_point(-4.5,-3.55)
    print('Reached 1st position!')
    #find(-4.5,-3.55)
    #go_to_point(1.6, -3.4)
    #print('Reached 2nd position')
    #find(1.6, -3.4)
    #print('Reached initial position')
    #go_to_point(0.0,0.0)
    print('Mission Complete')
    #rotate(1,2)
    #open_grip()
    #pick_up()
    #grab_up()
    #close_grip() 
    #pick()
    

    '''
        # Customize the following values so they are appropriate for your location
    position = {'x': 4.5, 'y' : 0.38}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

    success = goto(position, quaternion)

    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base failed to reach the desired pose")

    # Sleep to give the last log messages time to be sent
    #rospy.sleep(1)
    '''
    #open_grip()
    
    
    #except rospy.ROSInterruptException:
        #rospy.loginfo("Ctrl-C caught. Quitting")

    '''
    try:
       # Testing our function
        #rotate()
        find()
        open_grip()
        pick_up()
        grab_up()
        close_grip() 
        pick()
    except rospy.ROSInterruptException:
        pass
    '''

    '''
    global vel, pub_vel

    # intialize the node
    rospy.init_node('drive_wheel', anonymous=True)

    # subscribe to /ball_location topic to receive coordinates
    img_sub = rospy.Subscriber("/coordinates",Point, drive_callback)
    
    # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.spin()
    '''

    '''
#velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #vel_msg = Twist()
    angle = 360
    clockwise = 1

  #Converting from angles to radians
  #angular_speed = speed*2*PI/360
    angular_speed = 30*2*PI/360
    relative_angle = angle*2*PI/360


  #We wont use linear components
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0
    vel.angular.x = 0
    vel.angular.y = 0


# Checking if our movement is CW or CCW
    if clockwise:
            #vel_msg.angular.z = -abs(angular_speed)
        vel.angular.z = 0.5
    else:
        vel.angular.z = 0.5
            #vel_msg.angular.z = abs(angular_speed)

         # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    temp_count = 0


    while current_angle < relative_angle:
        pub_vel.publish(vel)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
        print('current_angle', current_angle)
        print('relative_angle ', relative_angle)
        r.sleep()
        if ball_x < 0 and ball_y < 0:
            print('ball_x check', ball_x, ball_y)
            continue
        else:
            break
        ''' 
