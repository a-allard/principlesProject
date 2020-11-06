#!/usr/bin/env python
import rospy
from ur10kinematics import urKinematics
from gripperController import gripper
import sys
import math
import roslib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import PyKDL

class ur10Arm:

    def __init__(self, urVersion='ur10'):
        self.rate1Hz = rospy.Rate(1)
        self.interface = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
        self.jointNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        self.ivk = urKinematics(urVersion)
        self.gripper = gripper()
        
        self.trajectoryMsg = JointTrajectory()
        self.trajectoryMsg.joint_names = self.jointNames
        
        self.trajecotryPointMsg = JointTrajectoryPoint()
        zeros = [0 for i in range(6)]
        self.trajecotryPointMsg.accelerations = zeros
        self.trajecotryPointMsg.velocities = zeros
        self.trajecotryPointMsg.time_from_start = rospy.Duration(0.5)
        
        
    
    def __publish__(self, jointAngles):
        self.trajecotryPointMsg.positions = jointAngles
        self.trajectoryMsg.points = [self.trajecotryPointMsg]
        for i in range(5):
            self.trajectoryMsg.header.stamp = rospy.Time.now()
            self.interface.publish(self.trajectoryMsg)
            self.rate1Hz.sleep()
            
    def setArmPosition(self, pos, preferedAngs=None):
        angs = self.ivk.inv_kin(pos, preferedAngs)
        self.__publish__(angs)
        