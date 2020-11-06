#!/usr/bin/env python
# This was written by Andrew Allard based on this file:
# https://github.com/littleblakew/UR_kinematics_solver/blob/master/kinematics.py

import numpy as np
import tf.transformations as tf
from math import sin, cos, pi, sqrt, radians, tan, acos, asin, atan2, degrees
import cmath
from geometry_msgs.msg import Pose, Quaternion

class urKinematics(object):

    def __init__(self, bot='ur10'):
        self.bot = bot.upper()
        self.initRobot()
        self.lastAngles = [0,-pi/2,0,0,0,0]
        
    
    def initRobot(self):
        if self.bot == 'UR10':
    
            # d (unit: mm)
            self.d1 = 0.1273
            self.d2 = self.d3 = 0
            self.d4 = 0.163941
            self.d5 = 0.1157
            self.d6 = 0.0922

            # a (unit: mm)
            self.a1 = self.a4 = self.a5 = self.a6 = 0
            self.a2 = -0.612
            self.a3 = -0.5723

        elif self.bot == 'UR5':

            # d (unit: mm)
            self.d1 = 0.089159 
            self.d2 = self.d3 = 0
            self.d4 = 0.10915
            self.d5 = 0.09465
            self.d6 = 0.0823

            # a (unit: mm)
            self.a1 = self.a4 = self.a5 = self.a6 = 0
            self.a2 = -0.425
            self.a3 = -0.39225

        elif self.bot == 'UR3':

            # d (unit: mm)
            self.d1 = 0.1519 
            self.d2 = self.d3 = 0
            self.d4 = 0.11235
            self.d5 = 0.08535
            self.d6 = 0.0819

            # a (unit: mm)
            self.a1 = self.a4 = self.a5 = self.a6 = 0
            self.a2 = -0.24365
            self.a3 = -0.21325

        else:
            
            # d (unit: mm)
            self.d1 = 0.1273
            self.d2 = self.d3 = 0
            self.d4 = 0.163941
            self.d5 = 0.1157
            self.d6 = 0.0922

            # a (unit: mm)
            self.a1 = self.a4 = self.a5 = self.a6 = 0
            self.a2 = -0.612
            self.a3 = -0.5723


        # List type of D-H parameter
        # Do not remove these
        self.d = np.array([self.d1, self.d2, self.d3, self.d4, self.d5, self.d6]) # unit: mm
        self.a = np.array([self.a1, self.a2, self.a3, self.a4, self.a5, self.a6]) # unit: mm
        self.alpha = np.array([pi/2, 0, 0, pi/2, -pi/2, 0]) # unit: radian

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
        angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)
        direction = [i / angle for i in ur_pose[3:6]]
        np_T = tf.rotation_matrix(angle, direction)
        np_q = tf.quaternion_from_matrix(np_T)
        ros_pose.orientation.x = np_q[0]
        ros_pose.orientation.y = np_q[1]
        ros_pose.orientation.z = np_q[2]
        ros_pose.orientation.w = np_q[3]
        
        return ros_pose


    def ros2np(self, ros_pose):
        """Transform pose from ROS Pose format to np.array format.
        Args:
            ros_pose: A pose in ROS Pose format (type: Pose)
        Returns:
            An HTM (type: np.array).
        """

        # orientation
        np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y, \
                                        ros_pose.orientation.z, ros_pose.orientation.w])
        
        # position
        np_pose[0][3] = ros_pose.position.x
        np_pose[1][3] = ros_pose.position.y
        np_pose[2][3] = ros_pose.position.z

        return np_pose


    def np2ros(self, np_pose):
        """Transform pose from np.array format to ROS Pose format.
        Args:
            np_pose: A pose in np.array format (type: np.array)
        Returns:
            An HTM (type: Pose).
        """

        # ROS pose
        ros_pose = Pose()

        # ROS position
        ros_pose.position.x = np_pose[0, 3]
        ros_pose.position.y = np_pose[1, 3]
        ros_pose.position.z = np_pose[2, 3]

        # ROS orientation 
        np_q = tf.quaternion_from_matrix(np_pose)
        ros_pose.orientation.x = np_q[0]
        ros_pose.orientation.y = np_q[1]
        ros_pose.orientation.z = np_q[2]
        ros_pose.orientation.w = np_q[3]

        return ros_pose


    def select(self, q_sols, q_d, w=[1]*6):
        """Select the optimal solutions among a set of feasible joint value 
        solutions.
        Args:
            q_sols: A set of feasible joint value solutions (unit: radian)
            q_d: A list of desired joint value solution (unit: radian)
            w: A list of weight corresponding to robot joints
        Returns:
            A list of optimal joint value solution.
        """

        error = []
        for q in q_sols:
            error.append(sum([w[i] * (q[i] - q_d[i]) ** 2 for i in range(6)]))
        
        return q_sols[error.index(min(error))]


    def HTM(self, i, theta):
        """Calculate the HTM between two links.
        Args:
            i: A target index of joint value. 
            theta: A list of joint value solution. (unit: radian)
        Returns:
            An HTM of Link l w.r.t. Link l-1, where l = i + 1.
        """

        Rot_z = np.matrix(np.identity(4))
        Rot_z[0, 0] = Rot_z[1, 1] = cos(theta[i])
        Rot_z[0, 1] = -sin(theta[i])
        Rot_z[1, 0] = sin(theta[i])

        Trans_z = np.matrix(np.identity(4))
        Trans_z[2, 3] = self.d[i]

        Trans_x = np.matrix(np.identity(4))
        Trans_x[0, 3] = self.a[i]

        Rot_x = np.matrix(np.identity(4))
        Rot_x[1, 1] = Rot_x[2, 2] = cos(self.alpha[i])
        Rot_x[1, 2] = -sin(self.alpha[i])
        Rot_x[2, 1] = sin(self.alpha[i])

        A_i = Rot_z * Trans_z * Trans_x * Rot_x
            
        return A_i


    # Forward Kinematics

    def fwd_kin(self, theta, i_unit='r', o_unit='n'):
        """Solve the HTM based on a list of joint values.
        Args:
            theta: A list of joint values. (unit: radian)
            i_unit: Output format. 'r' for radian; 'd' for degree.
            o_unit: Output format. 'n' for np.array; 'p' for ROS Pose.
        Returns:
            The HTM of end-effector joint w.r.t. base joint
        """

        T_06 = np.matrix(np.identity(4))

        if i_unit == 'd':
            theta = [radians(i) for i in theta]
        
        for i in range(6):
            T_06 *= self.HTM(i, theta)

        if o_unit == 'n':
            return T_06
        elif o_unit == 'p':
            return self.np2ros(T_06)


    # Inverse Kinematics

    def inv_kin(self, p, q_d=None, i_unit='r', o_unit='r'):
        """Solve the joint values based on an HTM.
        Args:
            p: A pose.
            q_d: A list of desired joint value solution 
                (unit: radian).
            i_unit: Output format. 'r' for radian; 'd' for degree.
            o_unit: Output format. 'r' for radian; 'd' for degree.
        Returns:
            A list of optimal joint value solution.
        """
        if q_d is None:
            q_d = self.lastAngles

        # Preprocessing
        if type(p) == Pose: # ROS Pose format
            T_06 = self.ros2np(p)
        elif type(p) == list: # UR format
            T_06 = self.ros2np(self.ur2ros(p))
        else:
            T_06 = p

        if i_unit == 'd':
            q_d = [radians(i) for i in q_d]

        # Initialization of a set of feasible solutions
        theta = np.zeros((8, 6))
    
        # theta1
        P_05 = T_06[0:3, 3] - self.d6 * T_06[0:3, 2]
        phi1 = atan2(P_05[1], P_05[0])
        phi2 = acos(self.d4 / sqrt(P_05[0] ** 2 + P_05[1] ** 2))
        theta1 = [pi / 2 + phi1 + phi2, pi / 2 + phi1 - phi2]
        theta[0:4, 0] = theta1[0]
        theta[4:8, 0] = theta1[1]
    
        # theta5
        P_06 = T_06[0:3, 3]
        theta5 = []
        for i in range(2):
            theta5.append(acos((P_06[0] * sin(theta1[i]) - P_06[1] * cos(theta1[i]) - self.d4) / self.d6))
        for i in range(2):
            theta[2*i, 4] = theta5[0]
            theta[2*i+1, 4] = -theta5[0]
            theta[2*i+4, 4] = theta5[1]
            theta[2*i+5, 4] = -theta5[1]
    
        # theta6
        T_60 = np.linalg.inv(T_06)
        theta6 = []
        for i in range(2):
            for j in range(2):
                s1 = sin(theta1[i])
                c1 = cos(theta1[i])
                s5 = sin(theta5[j])
                theta6.append(atan2((-T_60[1, 0] * s1 + T_60[1, 1] * c1) / s5, (T_60[0, 0] * s1 - T_60[0, 1] * c1) / s5))
        for i in range(2):
            theta[i, 5] = theta6[0]
            theta[i+2, 5] = theta6[1]
            theta[i+4, 5] = theta6[2]
            theta[i+6, 5] = theta6[3]

        # theta3, theta2, theta4
        for i in range(8):  
            # theta3
            T_46 = self.HTM(4, theta[i]) * self.HTM(5, theta[i])
            T_14 = np.linalg.inv(self.HTM(0, theta[i])) * T_06 * np.linalg.inv(T_46)
            P_13 = T_14 * np.array([[0, -self.d4, 0, 1]]).T - np.array([[0, 0, 0, 1]]).T
            if i in [0, 2, 4, 6]:
                theta[i, 2] = -cmath.acos((np.linalg.norm(P_13) ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3)).real
                theta[i+1, 2] = -theta[i, 2]
            # theta2
            theta[i, 1] = -atan2(P_13[1], -P_13[0]) + asin(self.a3 * sin(theta[i, 2]) / np.linalg.norm(P_13))
            # theta4
            T_13 = self.HTM(1, theta[i]) * self.HTM(2, theta[i])
            T_34 = np.linalg.inv(T_13) * T_14
            theta[i, 3] = atan2(T_34[1, 0], T_34[0, 0])       

        theta = theta.tolist()

        # Select the most close solution
        q_sol = self.select(theta, q_d)
        self.lastAngles = q_sol
        # Output format
        if o_unit == 'r': # (unit: radian)
            print(q_sol)
            return q_sol
        elif o_unit == 'd': # (unit: degree)
            return [degrees(i) for i in q_sol]
