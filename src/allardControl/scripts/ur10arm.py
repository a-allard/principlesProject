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
from cameraInterface import cameraInterface
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Pose, Quaternion

from pykdl_utils.kdl_kinematics import KDLKinematics
import tf.transformations as tf

class ur10Arm:

    def __init__(self, urVersion='ur10'):
        self.rate1Hz = rospy.Rate(1)
        self.interface = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
        self.jointNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.forceLastJoint = True
        self.lastJointValue = np.pi/2
        
        robot = URDF.from_parameter_server()
        self.ivk = KDLKinematics(robot, 'base_link', 'wrist_3_link')
        self.gripper = gripper()
        self.cam = cameraInterface()
        
        self.trajectoryMsg = JointTrajectory()
        self.trajectoryMsg.joint_names = self.jointNames
        
        self.trajecotryPointMsg = JointTrajectoryPoint()
        zeros = [0 for i in range(6)]
        self.trajecotryPointMsg.accelerations = zeros
        self.trajecotryPointMsg.velocities = zeros
        self.trajecotryPointMsg.time_from_start = rospy.Duration(0.5)
    
    def findClosestMiddleBlock(self):
        return np.array([(x[0]-400)**2+(x[1]-400)**2 for x in self.cam.blockLocations]).argmin()
        
    def centerBlock(self):
        self.cam.liveScan=True
        x, y = self.cam.blockLocations[self.findClosestMiddleBlock()]
        attempts = 0
        while np.sqrt((x - 500) **2 + (y - 400)**2) > 10:
            print(self.cam.blockLocations)
            print('{0}\t{1}'.format(x, y))
            x = (x - 500) * 0.0009
            y = (y - 400) * 0.0009
            pos = self.currentPos.copy()
            pos[0] -= x
            pos[1] += y
            # print(pos)
            # print(self.currentPos)
            self.setArmPosition(pos)
            if attempts > 25:
                print("Cain't cener that un.")
            attempts += 1
            x, y = self.cam.blockLocations[self.findClosestMiddleBlock()]
        return {'x': self.currentPos[0], 'y': self.currentPos[1]}
        
    def ur2ros(self, ur_pose):
        """Transform pose from UR format to ROS Pose format.
        Args:
            ur_pose: A pose in UR format [px, py, pz, rx, ry, rz] 
            (type: list)
        Returns:
            An HTM (type: Pose).
        """

        # ROS pose
        ros_pose = Pose()

        # ROS position
        ros_pose.position.x = ur_pose[0]
        ros_pose.position.y = ur_pose[1]
        ros_pose.position.z = ur_pose[2]

        # Ros orientation
        # angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)
        # direction = [i / angle for i in ur_pose[3:6]]
        # np_T = tf.rotation_matrix(angle, direction)
        # np_q = tf.quaternion_from_matrix(np_T)
        np_q = tf.quaternion_from_euler(ur_pose[3], ur_pose[4], ur_pose[5])
        ros_pose.orientation.x = np_q[0]
        ros_pose.orientation.y = np_q[1]
        ros_pose.orientation.z = np_q[2]
        ros_pose.orientation.w = np_q[3]
        
        return ros_pose
        
    
    def __publish__(self, jointAngles):
        self.trajecotryPointMsg.positions = jointAngles
        self.trajectoryMsg.points = [self.trajecotryPointMsg]
        for i in range(5):
            self.trajectoryMsg.header.stamp = rospy.Time.now()
            self.interface.publish(self.trajectoryMsg)
            self.rate1Hz.sleep()
            
    def setArmPosition(self, pos, preferedAngs=None):
        self.currentPos = np.array(pos)
        if preferedAngs is None:
            preferedAngs = [[-4, -2.1415, -np.pi,-3.5,-np.pi,-np.pi], [4, 0, np.pi, 1.8, np.pi, np.pi]]
            angs = self.ivk.inverse(self.ur2ros(pos), self.currentPos, preferedAngs[0], preferedAngs[1], 1000) # inverse kinematics
            if angs is None:
                angs = self.ivk.inverse_search(self.ur2ros(pos), 10, *preferedAngs) # inverse kinematics
        else:
            angs = self.ivk.inverse_search(self.ur2ros(pos), 10, *preferedAngs) # inverse kinematics
        # if self.forceLastJoint:
        #     mul = 1
        #     if angs[1] > 0:
        #         mul = -1
        #     if angs[0] > np.pi/2:
        #         angs[-1] = mul*self.lastJointValue + (angs[0] - np.pi)
        #     else:
        #         angs[-1] = mul*self.lastJointValue + (angs[0] + np.pi)
        print(angs)
        self.__publish__(angs)
        