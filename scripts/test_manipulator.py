#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
#from apriltag_ros.msg import AprilTagDetectionArray
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Twist


class openManipulator:
    def __init__(self):
        # ROS NODE INIT
        rospy.init_node('moveit_routine_fix', anonymous=True)       
        # MOVEIT INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = "arm"
        self.group_name = "gripper"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.grouparm = moveit_commander.MoveGroupCommander(self.group_arm)

        # DEFINE PARAMETERS
        self.group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
        self.group.set_planning_time(5)                      # TIME TO PLANNING
        self.grouparm.set_goal_position_tolerance(0.1)       # GOAL TOLERANCE
        self.grouparm.set_planning_time(5)


    def home_pos(self):
        joint_goal = self.grouparm.get_current_joint_values()
        joint_goal[2] = 0
        self.grouparm.go(joints=joint_goal, wait=True)
        self.grouparm.stop()
        self.grouparm.clear_pose_targets()

    def rotate_joint1_90(self,j):
        joint_goal = self.grouparm.get_current_joint_values()
        print j/2
        joint_goal[2] = -j/2
        joint_goal[1] = j/2 + 0.50
        joint_goal[3] = j/2 - 0.30
        #joint_goal[1] = j
        self.grouparm.go(joints=joint_goal, wait=True)
        self.grouparm.stop()
        self.grouparm.clear_pose_targets()

    def pick(self, j = pi/2):
        joint_goal = self.grouparm.get_current_joint_values()
        joint_goal[2] = -j/2
        joint_goal[1] = 0/2
        joint_goal[3] = 0/2
        #joint_goal[1] = j
        self.grouparm.go(joints=joint_goal, wait=True)
        self.grouparm.stop()
        self.grouparm.clear_pose_targets()    


    def pick_up(self,j = pi/2):
        joint_goal = self.grouparm.get_current_joint_values()

        joint_goal[1] = -j/2 - 0.30
        joint_goal[2] = j/2 + 0.30
        joint_goal[3] = j/2 - 0.30

        '''
        joint_goal[2] = -j/2 + 0.30
        joint_goal[1] = j/2 + 0.50
        joint_goal[3] = j/2 - 0.80
        '''
        '''
        joint_goal[2] = -j/2
        joint_goal[1] = j/2 + 0.50
        joint_goal[3] = j/2 - 1.0
        '''
        self.grouparm.go(joints=joint_goal, wait=True)
        self.grouparm.stop()
        self.grouparm.clear_pose_targets()

    def grab_up(self,j = pi/2):
        joint_goal = self.grouparm.get_current_joint_values()
        
        joint_goal[1] = j/2 + 0.60
        joint_goal[2] = -j/2
        joint_goal[3] = -j/2 + 0.80
        #joint_goal[1] = j
        self.grouparm.go(joints=joint_goal, wait=True)
        self.grouparm.stop()
        self.grouparm.clear_pose_targets()
        print "Grab"   


    def open_grip(self):
        print "Open Grip"
        jv = self.group.get_current_joint_values()  
        jv = [0.019,0.019]
        print(jv)
        self.group.go(joints=jv, wait=True)
        self.group.stop()
        #self.group.clear_pose_targets()
        '''
        self.group.set_joint_value_target(jv)
        p = self.group.plan()
        self.group.execute(p)
        '''
    def close_grip(self):
        print "Close Grip"
        jv = self.group.get_current_joint_values() 
        jv = [-0.005,-0.005]
        print(jv)
        self.group.go(joints=jv, wait=True)
        #self.group.stop()
        #self.group.clear_pose_targets() 




if __name__ == '__main__':
    try:
        manip = openManipulator()
        raw_input("Press Enter to start!")

        #manip.home_pos()
        manip.open_grip()
        manip.pick_up()
        manip.grab_up()
        #manip.rotate_joint1_90(pi/2)
        manip.close_grip() 
        manip.pick()
        #manip.rotate_joint1_90(0)


    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass